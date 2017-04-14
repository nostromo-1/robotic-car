/**********************************************************************************
Cödigo fuente del proyecto de coche robótico
Fichero: sound.c
Fecha: 21/2/2017

Reproduce un fichero de audio en formato WAV


***********************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <pthread.h>
#include <stdbool.h>

#include <pigpio.h>
#include <alsa/asoundlib.h>

#define PCM_DEVICE "default"
#define MIXER_DEVICE "PCM"
#define NUMBER32(p) (*(p) + (*((p)+1)<<8) + (*((p)+2)<<16) + (*((p)+3)<<24))
#define NUMBER16(p) (*(p) + (*((p)+1)<<8))


extern volatile bool playing_audio, cancel_audio;
static int ampliPIN;


struct data {
    snd_pcm_t *pcm_handle;
    char *mem;
    size_t filelen;
};



static void cleanup_outer(void *p)
{
    playing_audio = false;
    cancel_audio = false;
}


static void cleanup_inner(void *p)
{
    struct data *arg;
    
    arg = p;
    if (cancel_audio) snd_pcm_drop(arg->pcm_handle);
    else snd_pcm_drain(arg->pcm_handle);
    snd_pcm_close(arg->pcm_handle);

    munmap(arg->mem, arg->filelen);
    free(p);
    gpioWrite(ampliPIN, PI_OFF);  // shutdown amplifier
}



/* Set volume; 0-100% */
void setVolume(int volume)
{
    static long min, max;
    static snd_mixer_t *handle;
    static snd_mixer_selem_id_t *sid;
    static snd_mixer_elem_t* elem;
    int rc;

    if (volume > 100) volume = 100;
    if (volume < 0) volume = 0;
    if (!elem) {
        rc = snd_mixer_open(&handle, 0);
        if (rc < 0) {
            fprintf(stderr, "Error al abrir mixer de audio: %s\n", snd_strerror(rc));    
            return;
        }
        snd_mixer_attach(handle, PCM_DEVICE);
        snd_mixer_selem_register(handle, NULL, NULL);
        rc = snd_mixer_load(handle);
        if (rc < 0) {
            fprintf(stderr, "Error al abrir mixer de audio: %s\n", snd_strerror(rc));    
            return;
        }
        snd_mixer_selem_id_alloca(&sid);
        snd_mixer_selem_id_set_index(sid, 0);
        snd_mixer_selem_id_set_name(sid, MIXER_DEVICE);
        elem = snd_mixer_find_selem(handle, sid);
        if (!elem) return;
        rc = snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
        if (rc < 0) {
            fprintf(stderr, "Error al leer volumen de audio: %s\n", snd_strerror(rc));   
            elem = NULL;            
            return;
        }        
    }
    if (elem) snd_mixer_selem_set_playback_volume_all(elem, min + volume*(max-min)/100);
}



void setupSound(int gpio)
{
    ampliPIN = gpio;
    gpioSetMode(ampliPIN, PI_OUTPUT);
    gpioWrite(ampliPIN, PI_OFF);     // shutdown amplifier
}



void* play_wav(void *filename)
{
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *params;
    snd_pcm_uframes_t frames;
    snd_pcm_format_t pcmFormat;
    snd_pcm_sframes_t tframes;

    struct data *thread_data;
    
    int fd, rest, rc;
    char *p, *mem;
    size_t filelen;
    uint16_t num16;
    uint32_t num32;
    unsigned int channels, rate, bitsPerSample, periodSize, frameSize;
 
 
    playing_audio = true;
    pthread_cleanup_push(cleanup_outer, NULL);
    fd = open(filename, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Error en fichero %s: %s\n", filename, strerror(errno));    
        pthread_exit(NULL);
    }
    /* Mapea sección inicial de 12 bytes */
    mem = p = mmap(NULL, 12, PROT_READ, MAP_SHARED, fd, 0);
    if (mem == MAP_FAILED) {
        fprintf(stderr, "Error al mapear fichero en memoria\n");
        close(fd);        
        pthread_exit(NULL);
    }
    if (strncmp(mem, "RIFF", 4)) {
        fprintf(stderr, "Error: no es fichero de audio\n");    
        close(fd);
        munmap(mem, 12);
        pthread_exit(NULL);
    }
    p += 4;
    filelen = 8 + NUMBER32(p);
    p += 4;
    if (strncmp(p, "WAVE", 4)) {
        fprintf(stderr, "Error: no es fichero de audio\n");    
        close(fd);
        munmap(mem, 12);
        pthread_exit(NULL);
    }
    
    munmap(mem, 12);
    if (filelen > 50000000) {
        fprintf(stderr, "Error: mal fichero de audio\n");    
        close(fd);
        pthread_exit(NULL);                
    }

    /* Sección inicial correcta. Mapea fichero completo */    
    mem = mmap(NULL, filelen, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (mem == MAP_FAILED) {
        fprintf(stderr, "Error al mapear fichero en memoria\n");
        pthread_exit(NULL);
    }
    p = mem + 12;
    if (strncmp(p, "fmt ", 4)) {
        fprintf(stderr, "Error: no es fichero de audio\n");    
        munmap(mem, filelen);
        pthread_exit(NULL);
    }
    p += 8;
    if (*p != 1) {
        fprintf(stderr, "Error: no es fichero de audio PCM\n");    
        munmap(mem, filelen);
        pthread_exit(NULL);
    }
    p += 2;
    channels = NUMBER16(p);
    p += 2;
    rate = NUMBER32(p);
    p += 10;
    bitsPerSample = NUMBER16(p);
    switch (bitsPerSample) {
            case 8: 
                pcmFormat = SND_PCM_FORMAT_U8;
                break;        
            case 16:
                pcmFormat = SND_PCM_FORMAT_S16_LE;
                break;
            default:
                fprintf(stderr, "Error: solo se admiten 8 bits o 16 bits\n");    
                munmap(mem, filelen);
                pthread_exit(NULL);            
    }
    
    p += 2;
    if (strncmp(p, "data", 4)) {
        fprintf(stderr, "Error: no hay datos de audio\n");    
        munmap(mem, filelen);
        pthread_exit(NULL);
    }    
        
    p += 8;  // p situado al comienzo de los datos de audio
    
    /* Comienza el volcado de los datos de audio */
    
    /* Open the PCM device in playback mode */
    rc = snd_pcm_open(&pcm_handle, PCM_DEVICE, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
    if (rc < 0) {
        fprintf(stderr, "Error: no se puede abrir el dispositivo de audio: %s\n", snd_strerror(rc));    
        munmap(mem, filelen);
        pthread_exit(NULL);                    
    }

    /* Prepara datos para el inner cleanup handler */
    thread_data = malloc(sizeof(struct data));
    if (thread_data == NULL) {
        fprintf(stderr, "Error al reservar memoria\n");    
        munmap(mem, filelen);
        pthread_exit(NULL);                
    }
    thread_data->pcm_handle = pcm_handle;
    thread_data->mem = mem;
    thread_data->filelen = filelen;
    pthread_cleanup_push(cleanup_inner, thread_data);
    /* cleanup handler listo */
    
    snd_pcm_nonblock(pcm_handle, 0);  // Set blocking IO
        
    /* Allocate parameters object and fill it with default values */
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    
    /* Set parameters */
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, pcmFormat);
    snd_pcm_hw_params_set_channels(pcm_handle, params, channels);
    snd_pcm_hw_params_set_rate(pcm_handle, params, rate, 0);

    /* Write parameters */
    snd_pcm_hw_params(pcm_handle, params);

    /* Get number of frames in a single period */
    snd_pcm_hw_params_get_period_size(params, &frames, NULL);

    frameSize = channels * bitsPerSample/8;
    periodSize = frames * frameSize;

    gpioWrite(ampliPIN, PI_ON);   // Activate amplifier
    gpioSleep(PI_TIME_RELATIVE, 0, 100000);  // Wait for ampli to stabilize
    /* Bucle enviando datos de audio */
    while (rest = mem + filelen - p, rest>0) {
        if (cancel_audio) break;
        tframes = snd_pcm_writei(pcm_handle, p, rest>=periodSize?frames:rest/frameSize);    
        if (tframes < 0) tframes = snd_pcm_recover(pcm_handle, tframes, 0);
        if (tframes < 0) {
            fprintf(stderr, "Error al enviar datos de audio: %s\n", snd_strerror(tframes));
            break;
        }        
        p += periodSize;    
    }
    
    pthread_cleanup_pop(1);   // Quita y ejecuta el cleanup inner handler
    pthread_cleanup_pop(1);   // Quita y ejecuta el cleanup outer handler
    pthread_exit(NULL);
}


