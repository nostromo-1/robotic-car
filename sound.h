#ifndef SOUND_H
#define SOUND_H

/*************************************************************************
Control of the audio amplifier

*****************************************************************************/

// Inicializa el interfaz de audio. Parámetro: el pin por el que se activa el amplificador
void setupSound(int gpio);

// Ajusta el volumen de audio; rango: 0-100
void setVolume(int volume);

// Función para reproducir un fichero WAV de audio en un thread propio
void* play_wav(void *filename);  // Función en fichero sound.c



#endif // SOUND_H
