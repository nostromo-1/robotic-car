/**********************************************************************************
Cödigo fuente del proyecto de coche robótico
Fichero: motor.c
Fecha: 21/2/2017

Realiza el control principal del coche


***********************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <malloc.h>
#include <string.h>
#include <stdbool.h>
  
#include <pigpio.h>
#include <cwiid.h>
#include "oled96.h"
#include "sound.h"
#include "pcf8591.h"


extern char *optarg;
extern int optind, opterr, optopt;

/************ Define pin numbers (Broadcom pin number scheme) *********************/
#define MI_ENA 17
#define MI_IN1 27
#define MI_IN2 22

#define MD_ENA 16
#define MD_IN1 20
#define MD_IN2 21

#define SONAR_TRIGGER 23
#define SONAR_ECHO    24

#define PITO_PIN   26
#define VBAT_PIN   19
#define WMSCAN_PIN 12
#define AUDR_PIN   18
#define AUDR_ALT PI_ALT5   /* ALT function for audio pin, ALT5 for pin 18 */
#define AMPLI_PIN  25
#define LSENSOR_PIN 5
#define RSENSOR_PIN 6
#define KARR_PIN    4


/***************** Define constants and parameters ****************/
#define DISTMIN 50        /* distancia en cm a la que entendemos que hay un obstáculo */
#define INITIAL_SPEED 50  /* Entre 0 y 100% */
#define NUMPOS 3     /* Número de medidas de posición del sonar para promediar */
#define NUMHOLES 21  /* Número de agujeros en discos de encoder del motor */

#define PCF8591_I2C 0x48           /* Dirección i2c del PCF8591 (AD/DA converter) */
#define DISPLAY_I2C 0x3C           /* Dirección i2c del display SSD1306 */
#define LSM9DS1_GYR_ACEL_I2C 0x6B  /* Dirección i2c del módulo acelerómetro/giroscopio del IMU LSM9DS1 */
#define LSM9DS1_MAG_I2C 0x1E       /* Dirección i2c del módulo magnetómetro del IMU LSM9DS1 */




/* Callbacks called when some GPIO pin changes */
void speedSensor(int gpio, int level, uint32_t tick);
void wmScan(int gpio, int level, uint32_t tick);

 
/****************** Variables y tipos globales **************************/

typedef enum {ADELANTE, ATRAS} Sentido_t;

typedef struct {
    const unsigned int en_pin, in1_pin, in2_pin, sensor_pin;  /* Pines  BCM */
    volatile Sentido_t sentido;   /* ADELANTE, ATRAS */
    volatile int velocidad;       /* 0 a 100, velocidad (no real) impuesta al motor */
    volatile int PWMduty;         /* Valor de PWM para alcanzar la velocidad objetivo, 0-100 */
    volatile uint32_t counter, tick, rpm;
    pthread_mutex_t mutex;
} Motor_t;


typedef struct {
    cwiid_wiimote_t *wiimote;
    volatile uint16_t buttons;
} MandoWii_t;


typedef struct {
    const unsigned int pin;
    volatile bool pitando;
    pthread_mutex_t mutex;
} Bocina_t;


typedef struct {
    const unsigned int trigger_pin, echo_pin;
    volatile bool triggered;
    volatile uint32_t distancia;
} SonarHCSR04_t;


volatile uint32_t distancia = UINT32_MAX;
volatile int velocidadCoche = INITIAL_SPEED;  // velocidad objetivo del coche. Entre 0 y 100; el sentido de la marcha viene dado por el botón pulsado (A/B)
volatile int soundVolume = 96;  // 0 - 100%
volatile int powerState = PI_ON;
volatile bool esquivando; // tareas sincronizadas por el semáforo semaphore
volatile bool playing_audio, cancel_audio;   // Variables compartidas con fichero sound.c
volatile int karr_delay = 150000;  // Time in us to wait between leds in KARR scan

sem_t semaphore;
bool remoteOnly, useEncoder, checkBattery, softTurn, calibrateIMU; // program line options
char *alarmFile = "sounds/police.wav";
MandoWii_t mando;


Bocina_t bocina = {
    .pitando = false,
    .pin = PITO_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};

SonarHCSR04_t sonarHCSR04 = {
    .trigger_pin = SONAR_TRIGGER,
    .echo_pin = SONAR_ECHO,
    .distancia = UINT32_MAX,
    .triggered = false
};


Motor_t m_izdo = {
    .en_pin = MI_ENA,
    .in1_pin = MI_IN1,
    .in2_pin = MI_IN2,
    .sensor_pin = LSENSOR_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};

Motor_t m_dcho = {
    .en_pin = MD_ENA,
    .in1_pin = MD_IN1,
    .in2_pin = MD_IN2,
    .sensor_pin = RSENSOR_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};




/****************** Funciones del display **************************/

void oledBigMessage(int line, const char *msg)
{
    static const char *empty = "        ";
    char *buf;
    
    if (line<0 || line>1) {
        fprintf(stderr, "oledBigMessage: line must be 0 or 1\n");
        return;
    }
    if (msg) buf = (char *)msg; 
    else buf = (char *)empty;
    
    oledWriteString(0, 2+3*line, buf, true); 
}




/****************** Funciones de control de los motores **************************/
void ajustaSentido(Motor_t *motor, Sentido_t dir)
{
    switch(dir) {
      case ADELANTE:
        gpioWrite(motor->in1_pin, PI_OFF);
        gpioWrite(motor->in2_pin, PI_ON);
        break;
      case ATRAS:
        gpioWrite(motor->in1_pin, PI_ON);
        gpioWrite(motor->in2_pin, PI_OFF);
        break;
    }
    motor->sentido = dir;
}


void fastStopMotor(Motor_t *motor)
{
    pthread_mutex_lock(&motor->mutex);
    gpioWrite(motor->in1_pin, PI_OFF);
    gpioWrite(motor->in2_pin, PI_OFF);
    motor->PWMduty = motor->velocidad = 0;
    gpioPWM(motor->en_pin, motor->PWMduty);
    pthread_mutex_unlock(&motor->mutex);  
}


/* v va de 0 a 100 */
void ajustaMotor(Motor_t *motor, int v, Sentido_t sentido)
{    
    if (v > 100) v = 100;
    if (v < 0) {
        fprintf(stderr, "Error en ajustaMotor: v<0!\n");
        v = 0;
    }
    if (motor->velocidad == v && motor->sentido == sentido) return;
    
    pthread_mutex_lock(&motor->mutex);
    ajustaSentido(motor, sentido);
    motor->PWMduty = motor->velocidad = v;
    gpioPWM(motor->en_pin, motor->PWMduty);
    pthread_mutex_unlock(&motor->mutex);   
}



/* Rota el coche a la derecha (dextrógiro, sentido>0) o a la izquierda (levógiro, sentido<0)*/
void rota(Motor_t *izdo, Motor_t *dcho, int sentido)  
{
    if (sentido>0) {  // sentido horario
       ajustaMotor(izdo, softTurn?50:100, ADELANTE);
       ajustaMotor(dcho, softTurn?0:100, ATRAS);
    } else{
       ajustaMotor(dcho, softTurn?50:100, ADELANTE);
       ajustaMotor(izdo, softTurn?0:100, ATRAS);
    }    
}


int setupMotor(Motor_t *motor)
{
    int r = 0;
    
    r |= gpioSetMode(motor->in1_pin, PI_OUTPUT);
    r |= gpioSetMode(motor->in2_pin, PI_OUTPUT);
    
    if (gpioSetPWMfrequency(motor->en_pin, 100)<0) r = -1;   /* 100 Hz, low but not audible */
    if (gpioSetPWMrange(motor->en_pin, 100)<0) r = -1;       /* Range: 0-100, real range = 2000 */
    
    if (useEncoder) {
        r |= gpioSetMode(motor->sensor_pin, PI_INPUT);
        gpioSetPullUpDown(motor->sensor_pin, PI_PUD_DOWN);
        gpioSetAlertFunc(motor->sensor_pin, speedSensor);
        gpioGlitchFilter(motor->sensor_pin, 1000);      
    }
    
    if (r) fprintf(stderr, "Cannot initialise motor!\n");
    return r;
}



/****************** Funciones de control del sensor de distancia de ultrasonidos HC-SR04 **************************/

/* trigger a sonar reading */
void sonarTrigger(void)
{
   gpioWrite(sonarHCSR04.trigger_pin, PI_ON);
   gpioDelay(10);     /* 10us trigger pulse */
   gpioWrite(sonarHCSR04.trigger_pin, PI_OFF);
   sonarHCSR04.triggered = true;
}


/* callback llamado cuando el pin SONAR_ECHO cambia de estado. Ajusta la variable global 'distancia' */
void sonarEcho(int gpio, int level, uint32_t tick)
{
   static uint32_t startTick, distance_array[NUMPOS], pos_array=0;
   static uint32_t distancia_previa = UINT32_MAX;
   static int firstTime=0;
   static bool false_echo;
   uint32_t suma, diffTick, d;
   char str[17];
   int i;
  
   switch (level) {
   case PI_ON: 
           if (sonarHCSR04.triggered) {    
               startTick = tick;
               false_echo = false;
           } else false_echo = true;
           return;  // Not break; should not execute further after this switch

   case PI_OFF: 
           if (false_echo) return;  // Not break
           diffTick = tick - startTick;  // pulse length in microseconds
           if (diffTick > 23000 || diffTick < 60) break;  /* out of range */
           d = (diffTick*17)/1000;  // distance in cm

           distance_array[pos_array++] = d;
           if (pos_array == NUMPOS) pos_array = 0;
 
           if (firstTime>=0) {
              if (firstTime < NUMPOS) { /* The first NUMPOS times until array is filled */
                  //distancia = d;
                  firstTime++;
                  break;
              } else firstTime = -1;  /* Initialisation is over */
           }
           
           /* Calculate moving average */
           for (i=0, suma=0; i<NUMPOS; i++) suma += distance_array[i]; 
           sonarHCSR04.distancia = suma/NUMPOS;   
           
           /* Set global variable "distancia" and activate main loop if below threshold */
           distancia = sonarHCSR04.distancia;  
           if (distancia != distancia_previa) {
               snprintf(str, sizeof(str), "Dist (cm): %-3u", distancia);
               oledWriteString(0, 0, str, false);
               distancia_previa = distancia;
           }
           
           if (!remoteOnly && distancia < DISTMIN && !esquivando) {
               esquivando = true;
               i = sem_post(&semaphore);  // indica al loop de main que hay un obstáculo delante
               if (i) perror("Error al activar semáforo");
           }
           break;
   } 
   // Only executed after PI_OFF
   sonarHCSR04.triggered = false;
}


int setupSonarHCSR04(void)
{
   gpioSetMode(sonarHCSR04.trigger_pin, PI_OUTPUT);
   gpioWrite(sonarHCSR04.trigger_pin, PI_OFF);
   gpioSetMode(sonarHCSR04.echo_pin, PI_INPUT);

   /* update sonar several times a second */
   if (gpioSetTimerFunc(0, 60, sonarTrigger) ||     /* trigger sonar every 60ms, timer#0 */
        gpioSetAlertFunc(sonarHCSR04.echo_pin, sonarEcho)) {  /* monitor sonar echos */
        fprintf(stderr, "Error al inicializar el sonar!\n");
        return -1;
       }
   return 0;
}


void closeSonarHCSR04(void)
{
   gpioSetTimerFunc(0, 60, NULL);
   gpioSetAlertFunc(sonarHCSR04.echo_pin, NULL);
   gpioSetMode(sonarHCSR04.trigger_pin, PI_INPUT);
}


/****************** Funciones de la bocina **************************/
static void activaPito(void)
{
    pthread_mutex_lock(&bocina.mutex);
    if (bocina.pitando == 0) gpioWrite(bocina.pin, PI_ON);
    bocina.pitando++;
    pthread_mutex_unlock(&bocina.mutex);
}


static void desactivaPito(void)
{
    pthread_mutex_lock(&bocina.mutex);
    if (bocina.pitando > 0) bocina.pitando--;
    if (bocina.pitando == 0) gpioWrite(bocina.pin, PI_OFF);
    pthread_mutex_unlock(&bocina.mutex);
}


/*  Función interna auxiliar */
static void* duerme_pitando(void *arg)
{
    uint32_t decimas, s, m;
    
    decimas = *(uint32_t*)arg;
    s = decimas/10;
    m = 100000*(decimas%10);
    activaPito();
    gpioSleep(PI_TIME_RELATIVE, s, m);   
    desactivaPito();
    free(arg);
    return NULL;
}


/* Toca el pito durante un tiempo (en decimas de segundo) 
modo=0; pita en otro hilo; vuelve inmediatamente 
modo=1; pita en este hilo, vuelve después de haber pitado */
void pito(uint32_t decimas, int modo)
{
    pthread_t pth;
    uint32_t *pd;
        
    if (decimas == 0) return;
    pd = malloc(sizeof(uint32_t));
    if (!pd) return;
    *pd = decimas;
    if (modo == 0) {
        if (pthread_create(&pth, NULL, duerme_pitando, pd)) {
            free(pd);
            return;
        }
        pthread_detach(pth);
    } else duerme_pitando(pd);
}



/******************Funciones de audio ************************/


/* Reproduce "file" en otro hilo.
file debe ser un string invariable, en memoria 
modo=0; vuelve inmediatamente, sin esperar el final
modo=1; vuelve después de haber tocado */
void audioplay(char *file, int modo)
{
    pthread_t pth;
    
    /* Si ya estamos reproduciendo algo, manda señal de cancelación al thread de audio */
    if (playing_audio) {
        cancel_audio = true;
        return;
    }
    if (pthread_create(&pth, NULL, play_wav, file)) return;
    if (modo == 0) pthread_detach(pth); 
    else pthread_join(pth, NULL);    
}




/****************** Funciones del Wiimote **************************/
void wiiErr(cwiid_wiimote_t *wiimote, const char *s, va_list ap)
{
  if (wiimote) fprintf(stderr, "Wiimote %d:", cwiid_get_id(wiimote)); 
  else fprintf(stderr, "Wiimote:");
  vprintf(s, ap);
  printf("\n");
}


/*** wiimote event loop ***/
static void wiiCallback(cwiid_wiimote_t *wiimote, int mesg_count, union cwiid_mesg mesg[], struct timespec *t)
{
    int i, v_izdo, v_dcho;
    Sentido_t s_izdo, s_dcho;
    static uint16_t previous_buttons;
    unsigned int bateria;
    static int LEDs[4] = { CWIID_LED1_ON,  CWIID_LED1_ON | CWIID_LED2_ON,
                CWIID_LED1_ON | CWIID_LED2_ON | CWIID_LED3_ON,
                CWIID_LED1_ON | CWIID_LED2_ON | CWIID_LED3_ON | CWIID_LED4_ON };        

     for (i = 0; i < mesg_count; i++) {
        switch (mesg[i].type) {
        case CWIID_MESG_BTN:  // Change in buttons
            mando.buttons = mesg[i].btn_mesg.buttons;

            // Save data from accelerometer in a file. Start and end saving when '2' is pressed
            if (previous_buttons&CWIID_BTN_2 && ~mando.buttons&CWIID_BTN_2) save_accel_data();

            /*** Botones + y - ***/           
            if (previous_buttons&CWIID_BTN_PLUS && ~mando.buttons&CWIID_BTN_PLUS) {
                if (mando.buttons&CWIID_BTN_1) { /* sube volumen: buttons ´1´ + ´+´ */
                    soundVolume += 2;
                    if (soundVolume > 100) soundVolume = 100;
                    setVolume(soundVolume);
                } else {  /* ajusta la velocidad del coche y la marca en leds del mando */
                    velocidadCoche += 10;
                    if (velocidadCoche > 100) velocidadCoche = 100;
                    cwiid_set_led(wiimote, LEDs[velocidadCoche/26]);
                }
            }
            
            /*** Botones + y - junto con botón ´1´ ***/
            if (previous_buttons&CWIID_BTN_MINUS && ~mando.buttons&CWIID_BTN_MINUS) {
                 if (mando.buttons&CWIID_BTN_1) { /* baja volumen: buttons ´1´ + ´-´ */
                    soundVolume -= 2;
                    if (soundVolume < 0) soundVolume = 0;
                    setVolume(soundVolume);
                } else { /* ajusta la velocidad del coche y la marca en leds del mando */
                    velocidadCoche -= 10;
                    if (velocidadCoche < 0) velocidadCoche = 0;
                    cwiid_set_led(wiimote, LEDs[velocidadCoche/26]);
                }
            }
            
            /*** Botones A, B y RIGHT, LEFT; si estamos esquivando, sale: el loop de main tiene el control ***/
            if (!esquivando) {
                /*** Botones A y B, leen la variable global "velocidadCoche" ***/
                if (mando.buttons&(CWIID_BTN_A | CWIID_BTN_B)) { // if A or B or both pressed
                    v_izdo = v_dcho = velocidadCoche;
                    if (mando.buttons&CWIID_BTN_A) s_izdo = s_dcho = ADELANTE;
                    else s_izdo = s_dcho = ATRAS;  // si vamos marcha atrás (botón B), invierte sentido
    
                    /*** Botones LEFT y RIGHT, giran el coche ***/
                    if (mando.buttons&CWIID_BTN_RIGHT) {
                        s_dcho = 1 - s_dcho;
                        if (softTurn) v_dcho = 0;
                        else v_dcho = 50;
                        v_izdo += 20; 
                    } 
            
                    if (mando.buttons&CWIID_BTN_LEFT) {
                        s_izdo = 1 - s_izdo;
                        if (softTurn) v_izdo = 0; 
                        else v_izdo = 50;
                        v_dcho += 20;                
                    }
                } else {  // Ni A ni B pulsados
                    v_izdo = v_dcho = 0;
                    s_izdo = s_dcho = ADELANTE;
                }
                        
                /*** Ahora activa la velocidadCoche calculada en cada motor ***/
                ajustaMotor(&m_izdo, v_izdo, s_izdo);
                ajustaMotor(&m_dcho, v_dcho, s_dcho);
            }
    
            /*** pito ***/
            if (~previous_buttons&CWIID_BTN_DOWN && mando.buttons&CWIID_BTN_DOWN) activaPito();    
            if (previous_buttons&CWIID_BTN_DOWN && ~mando.buttons&CWIID_BTN_DOWN) desactivaPito();    
            
            
            /*** sonido ***/
            if (~previous_buttons&CWIID_BTN_UP && mando.buttons&CWIID_BTN_UP) {
                audioplay(alarmFile, 0);
            }
        
            /*** End of buttons loop ***/
            previous_buttons = mando.buttons;
            break;
                    
        case CWIID_MESG_STATUS:
            bateria = (100*mesg[i].status_mesg.battery)/CWIID_BATTERY_MAX;
            printf("Bateria del wiimote: %u\%\n", bateria);
            cwiid_set_led(wiimote, LEDs[velocidadCoche/26]);
            break;
            
        default:
            fprintf(stderr, "Mensaje desconocido del wiimote!!\n");
            break;
        }
    }
}


void setupWiimote(void)
{
    static uint8_t bluetooth_glyph[] = {0, 66, 36, 255, 153, 90, 36, 0};
    cwiid_wiimote_t *wiimote;
    bdaddr_t ba;
    
    if (mando.wiimote) cwiid_close(mando.wiimote);
    cwiid_set_err(wiiErr);
    mando.wiimote = NULL;
    mando.buttons = 0;
    oledSetBitmap8x8(15*8, 0, NULL);  // 15: last position in line (0-15), clear icon
    oledBigMessage(1, "Scan... ");
    pito(5, 1);   // Pita 5 décimas para avisar que comienza búsqueda de mando
    
    printf("Pulsa las teclas 1 y 2 en el mando de la Wii...\n");
    gpioSleep(PI_TIME_RELATIVE, 2, 0);  // para dar tiempo a desconectar el mando si estaba conectado
    ba = *BDADDR_ANY;
    wiimote = cwiid_open_timeout(&ba, 0, 5);  // 5 seconds timeout
    if (!wiimote ||
        cwiid_set_rpt_mode(wiimote, CWIID_RPT_BTN | CWIID_RPT_STATUS) || 
        cwiid_set_mesg_callback(wiimote, wiiCallback) ||
        cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC)) {
            fprintf(stderr, "No puedo conectarme al mando de la Wii!\n");
            mando.wiimote = NULL;
            oledBigMessage(1, NULL);
            return;  // No es error si no hay wiimote, el coche funciona sin mando
    } 
    
    // wiimote found
    mando.wiimote = wiimote;
    printf("Conectado al mando de la Wii\n");
    oledBigMessage(1, NULL);
    oledSetBitmap8x8(15*8, 0, bluetooth_glyph);
    cwiid_set_rumble(wiimote, 1);  // señala mediante zumbido el mando sincronizado
    gpioSleep(PI_TIME_RELATIVE, 0, 500000);   // Espera 0,5 segundos
    cwiid_set_rumble(wiimote, 0);
    return;
}


static void* scanWiimotes(void *arg)
{
bool *scanning = (bool*)arg;

    *scanning = true;  // signal to wmScan that scanning is already in place    
    ajustaMotor(&m_izdo, 0, ADELANTE);  // Para el coche mientras escanea wiimotes
    ajustaMotor(&m_dcho, 0, ADELANTE);
    velocidadCoche = INITIAL_SPEED;   // Nueva velocidad inicial, con o sin mando 
    oledWriteString(0, 1, "    ", false); // Borra mensaje de "Auto", si está           
    setupWiimote();   
    if (!mando.wiimote && !remoteOnly) {  // No hay mando, coche es autónomo
        oledWriteString(0, 1, "Auto", false);
        ajustaMotor(&m_izdo, velocidadCoche, ADELANTE);
        ajustaMotor(&m_dcho, velocidadCoche, ADELANTE);                
    } 
    *scanning = false;  // signal to wmScan that scanning is over
    return NULL;
}



/* callback llamado cuando el pin WMSCAN_PIN cambia de estado. Tiene un pull-up a VCC, OFF==pulsado */
void wmScan(int gpio, int level, uint32_t tick)
{
static uint32_t time;
static bool pressed;
static bool scanning;
pthread_t pth;
    
    switch (level) {
        case PI_ON:   // Sync button released
            if (!pressed) return;   // elimina clicks espureos
            pressed = false;
            
            /* Long press (>2 sec): shutdown */
            if (tick-time > 2*1000000) {
                oledBigMessage(1, "SHUTDOWN");
                pito(10, 1);
                execlp("halt", "halt", NULL);
                return; // should never return
            }
            
            /* Short press: scan wiimotes in another thread, in order to return quickly to caller */           
            if (!scanning && !pthread_create(&pth, NULL, scanWiimotes, &scanning)) pthread_detach(pth);    
            break;
            
        case PI_OFF:  // Sync button pressed
            pressed = true;
            time = tick;
            break;
    }
}



/***************Funciones de control de la velocidad ********************/
/* callback llamado cuando el pin LSENSOR_PIN o RSENSOR_PIN cambia de estado
Se usa para medir la velocidad de rotación de las ruedas */
void speedSensor(int gpio, int level, uint32_t tick)
{
Motor_t *motor;

    switch (gpio) {
        case LSENSOR_PIN: 
            motor = &m_izdo; 
            break;
        case RSENSOR_PIN: 
            motor = &m_dcho; 
            break;  
    }

    switch (level) {
        case PI_ON:
            motor->counter++;
            motor->tick = tick;
            break;
        case PI_OFF:
            motor->counter++;
            motor->tick = tick;          
            break;
    }    
}


/* Callback llamado regularmente. Realiza el lazo de control de la velocidad, comparando
la diferencia de velocidades entre los motores para igualarlas */
void speedControl(void)
{
static uint32_t past_lcounter, past_ltick;    
uint32_t current_lcounter =  m_izdo.counter, current_ltick = m_izdo.tick;  
uint32_t lpulses=0, lperiod, lfreq;

static uint32_t past_rcounter, past_rtick;    
uint32_t current_rcounter =  m_dcho.counter, current_rtick = m_dcho.tick;  
uint32_t rpulses=0, rperiod, rfreq;
  
int pv, kp=15;  // parameters of PID filter: pv is measured error (process value), kp is proportionality constant
  
    // Left motor
    if (past_ltick == 0) goto l_end;  // Exceptionally, it seems a good use of goto
    lperiod = current_ltick - past_ltick;
    if (lperiod) {
        lpulses = current_lcounter - past_lcounter;
        lfreq = (1000000*lpulses)/lperiod;
        m_izdo.rpm = lfreq*60/NUMHOLES/2;
        //printf("Left motor: pulses=%u, rpm=%u\n", lpulses, m_izdo.rpm);
    }
    
    l_end:
    if (past_ltick == 0 || lperiod == 0) {
        //printf("Left motor stopped\n");
        m_izdo.rpm = 0;
    }
    past_lcounter = current_lcounter;
    past_ltick = current_ltick;
    
    // Right motor
    if (past_rtick == 0) goto r_end;  // Exceptionally, it seems a good use of goto
    rperiod = current_rtick - past_rtick;
    if (rperiod) {
        rpulses = current_rcounter - past_rcounter;
        rfreq = (1000000*rpulses)/rperiod;
        m_dcho.rpm = rfreq*60/NUMHOLES/2;
        //printf("Right motor: pulses=%u, rpm=%u\n", rpulses, m_dcho.rpm);
    }
    
    r_end:
    if (past_rtick == 0 || rperiod == 0) {
        //printf("Right motor stopped\n");
        m_dcho.rpm = 0;    
    }
    past_rcounter = current_rcounter;
    past_rtick = current_rtick;
    
    /******* P control loop. SP=0, PV=lpulses-rpulses *********/
    if (m_izdo.velocidad>0 && m_izdo.rpm==0) printf("Left motor stalled\n");
    if (m_dcho.velocidad>0 && m_dcho.rpm==0) printf("Right motor stalled\n");   
    if (m_izdo.velocidad != m_dcho.velocidad) return;  // Enter control section if straight line desired: both speeds equal
    if (m_izdo.velocidad == 0) return;  // If speed is 0 (in both), do not enter control section
    pv = lpulses - rpulses;
    if (pv<=1 && pv >=-1) return;  // Tolerable error, do not enter control section
    
    /** Control section loop; adjust parameters of filter **/
    //printf("\tAdjust left: %i, right: %i\n", -(kp*pv)/10, (kp*pv)/10);
    pthread_mutex_lock(&m_izdo.mutex);
    pthread_mutex_lock(&m_dcho.mutex);   
    m_izdo.PWMduty -= (kp*pv)/10;
    m_dcho.PWMduty += (kp*pv)/10;
    if (m_izdo.PWMduty>100) m_izdo.PWMduty = 100;
    if (m_izdo.PWMduty<0) m_izdo.PWMduty = 0;
    if (m_dcho.PWMduty>100) m_dcho.PWMduty = 100;
    if (m_dcho.PWMduty<0) m_dcho.PWMduty = 0;
    gpioPWM(m_izdo.en_pin, m_izdo.PWMduty);
    gpioPWM(m_dcho.en_pin, m_dcho.PWMduty);   
    pthread_mutex_unlock(&m_izdo.mutex); 
    pthread_mutex_unlock(&m_dcho.mutex);
}




/****************** Funciones auxiliares varias **************************/


/* Comprueba si los motores tienen alimentación.
Devuelve valor de las baterías de alimentación del motor en la variable global powerState: 
PI_OFF si no hay alimentación o es baja, PI_ON si todo OK
Toma 3 muestras espaciadas 600 ms, en bucle, hasta que coincidan.
Esto es necesario porque al arrancar los motores hay un tiempo de oscilación de las baterías, 
aunque está amortiguado por el condensador del circuito */    
void getPowerState(void)
{
static uint8_t emptybatt_glyph[] = {0, 0, 254, 131, 131, 254, 0, 0};
int power1, power2, power3;
int n = 3;
    
    do {
        power1 = gpioRead(VBAT_PIN);
        if (power1 < 0) return;  // Error al leer, valor inválido
        gpioSleep(PI_TIME_RELATIVE, 0, 600000);  
        power2 = gpioRead(VBAT_PIN);
        if (power2 < 0) return;
        gpioSleep(PI_TIME_RELATIVE, 0, 600000);  
        power3 = gpioRead(VBAT_PIN);
        if (power3 < 0) return;
    } while (power1!=power2 || power2!=power3);
    powerState = power1;  // set global variable
    
    // señal acústica en caso de batería baja
    if (powerState == PI_OFF) {
        oledBigMessage(0, "Bateria!");
        oledSetBitmap8x8(14*8, 0, emptybatt_glyph);
        while(n--) {
            pito(2, 1);  // pita 2 décimas en este hilo (vuelve después de pitar)
            gpioSleep(PI_TIME_RELATIVE, 0, 200000);  // espera 2 décimas de segundo
        }
        gpioSleep(PI_TIME_RELATIVE, 2, 0);
        oledBigMessage(0, NULL);
    }
}




/* Thread in charge of sending a clock pulse to the circuit implementing the KARR scan effect.
At each LH transition, the led will change */
static void* karrScan(void *arg)
{
static int clock_state=PI_OFF;    

   for (;;) {    
      switch (clock_state) {
         case PI_ON: gpioSleep(PI_TIME_RELATIVE, 0, 100);  // set high state for clock during 100 us 
                     gpioWrite(KARR_PIN, PI_OFF); 
                     clock_state = PI_OFF;
                     break;
                 
         case PI_OFF: gpioSleep(PI_TIME_RELATIVE, 0, karr_delay);
                      gpioWrite(KARR_PIN, PI_ON);  // LH clock transition changes led
                      clock_state = PI_ON;
                      break;
      }   
   }   
}



/* Activa el efecto scanner de los leds delanteros, como KARR */
void activateKarr(void)
{
pthread_t pth;
    
    gpioSetMode(KARR_PIN, PI_OUTPUT);
    gpioWrite(KARR_PIN, PI_OFF);    
    if (!pthread_create(&pth, NULL, karrScan, NULL)) pthread_detach(pth);   
}



/* Para el coche, cierra todo y termina el programa */
void terminate(int signum)
{
   fastStopMotor(&m_izdo);
   fastStopMotor(&m_dcho);
   if (mando.wiimote) cwiid_close(mando.wiimote);
   closeSonarHCSR04();
   closeLSM9DS1();
   gpioSetPullUpDown(WMSCAN_PIN, PI_PUD_OFF);
   gpioSetPullUpDown(VBAT_PIN, PI_PUD_OFF); 
   gpioSetPullUpDown(m_izdo.sensor_pin, PI_PUD_OFF);   
   gpioSetPullUpDown(m_dcho.sensor_pin, PI_PUD_OFF); 
   gpioWrite(bocina.pin, PI_OFF);
   gpioSetMode(AUDR_PIN, PI_INPUT);
   gpioSetMode(AMPLI_PIN, PI_INPUT);
   gpioSetMode(KARR_PIN, PI_INPUT);
    
   oledShutdown();
   gpioTerminate();
   exit(1);
}



int setup(void)
{
   int r = 0;
    
   if (gpioCfgClock(5, PI_CLOCK_PCM, 0)<0) return 1;   /* Standard settings: Sample rate: 5 us, PCM clock */
   gpioCfgInterfaces(PI_DISABLE_FIFO_IF | PI_DISABLE_SOCK_IF);
   if (gpioInitialise()<0) return 1;
   if (gpioSetSignalFunc(SIGINT, terminate)<0) return 1;
   
   // Inicializa display
   oledInit(DISPLAY_I2C);
   oledSetInversion(true);   // Fill display, as life sign
      
   // Inicializa altavoz y bocina
   setupSound(AMPLI_PIN);
   setVolume(soundVolume);
   gpioSetMode(AUDR_PIN, AUDR_ALT);  // Saca PWM0 (audio right) por el GPIO al amplificador
   gpioSetMode(bocina.pin, PI_OUTPUT);
   gpioWrite(bocina.pin, PI_OFF);
   
   if (checkBattery) {
      r = setupPCF8591(PCF8591_I2C);
      if (r==0) gpioSetTimerFunc(1, 500, checkPowerPCF); // Comprueba tensión baterías cada 500 mseg, timer#1
      else { // ADC not available, check battery via VBAT_PIN  
         gpioSetMode(VBAT_PIN, PI_INPUT);
         gpioSetPullUpDown(VBAT_PIN, PI_PUD_DOWN);   // pull-down resistor; avoids floating pin if circuit not connected  
         gpioSetTimerFunc(1, 30000, getPowerState);  // Comprueba tensión baterías cada 30 seg, timer#1
         getPowerState();
      }
   }

   gpioSetMode(WMSCAN_PIN, PI_INPUT);
   gpioSetPullUpDown(WMSCAN_PIN, PI_PUD_UP);  // pull-up resistor; button pressed == OFF
   gpioGlitchFilter(WMSCAN_PIN, 100000);      // 0,1 sec filter
   
   r |= setupMotor(&m_izdo);
   r |= setupMotor(&m_dcho);
   r |= sem_init(&semaphore, 0, 0);
   
   if (powerState == PI_OFF) {
       fprintf(stderr, "La bateria esta descargada. Coche no arranca!\n");
       terminate(SIGINT);
   }   

   setupLSM9DS1(LSM9DS1_GYR_ACEL_I2C, LSM9DS1_MAG_I2C, calibrateIMU);   // Setup IMU
   setupWiimote(); 
   gpioSetAlertFunc(WMSCAN_PIN, wmScan);  // Call wmScan when button changes. Debe llamarse después de setupWiimote
   if (useEncoder) gpioSetTimerFunc(2, 200, speedControl);  // Control velocidad motores, timer#2
   
   activateKarr();  // start KARR scanner effect
   oledSetInversion(false); // clear display
   
   r |= setupSonarHCSR04();

   return r;
}


/****************** Main **************************/
void main(int argc, char *argv[])
{
   int r;
   double volts;
   
   opterr = 0;  // Prevent getopt from outputting error messages
   while ((r = getopt(argc, argv, "crbesf:")) != -1)
       switch (r) {
           case 'r':  /* Remote only mode: does not measure distance */
               remoteOnly = true;
               break;
           case 'b':  /* Check battery */
               checkBattery = true;
               break;          
           case 'e':  /* Use wheel encoders */
               useEncoder = true;
               break;    
           case 's':  /* Soft turning (for 2WD) */
               softTurn = true;
               break;                 
           case 'f': /* Set sound file to be played with UP arrow */
               alarmFile = optarg;
               break;
           case 'c': /* Calibrate IMU sensor */
               calibrateIMU = true;
               break;
           default:
               fprintf(stderr, "Uso: %s [-r] [-b] [-e] [-s] [-f <fichero de alarma>]\n", argv[0]);
               exit(1);
   }
   
   r = setup();
   if (r) {
       fprintf(stderr, "Error al inicializar. Coche no arranca!\n");
       if (r < 0) terminate(SIGINT);   // Si el error estaba al inicializar pigpio (r>0), no llames a terminate
       else exit(1);
   }
  
   oledBigMessage(0, " Ready  ");
   audioplay("sounds/ready.wav", 1);
   oledBigMessage(0, NULL);
   
   // Check if battery low; -1 means that the ADC does not work correctly
   volts = getMainPowerValue();  
   if (checkBattery && volts>=0 && volts<6.6) {
      oledBigMessage(0, "Bateria!");
      audioplay("sounds/batterylow.wav", 1);
      oledBigMessage(0, NULL);           
   }
   
   if (!mando.wiimote && !remoteOnly) {  // No hay mando, el coche es autónomo
        oledWriteString(0, 1, "Auto", false);
        ajustaMotor(&m_izdo, velocidadCoche, ADELANTE);
        ajustaMotor(&m_dcho, velocidadCoche, ADELANTE);       
   }
   
   for (;;) { 
       esquivando = false;  // señala a sonarEcho que ya se puede volver a enviar la señal de obstáculo encontrado
       r = sem_wait(&semaphore);   // bloquea hasta que encontremos un obstáculo o haya que escanear wiimotes
       if (r) {
            perror("Error al esperar al semaforo");
            continue;
       }
             
       if (mando.wiimote && ~mando.buttons&CWIID_BTN_A) continue;  // Enter loop only if A is pressed
     
       /* Code gets here when obstacle found and robot is not already avoiding it */
       oledBigMessage(0, "OBSTACLE");         
       
       // Loop for autonomous mode 
       while (distancia < DISTMIN) {
            //printf("Distancia: %d cm\n", distancia); 
            fastStopMotor(&m_izdo);
            fastStopMotor(&m_dcho);
            gpioSleep(PI_TIME_RELATIVE, 1, 0);
       
            // No esquiva si ya no pulsamos A
            if (mando.wiimote && ~mando.buttons&CWIID_BTN_A) break;

            // Gira un poco el coche para esquivar el obstáculo
            if (mando.wiimote && mando.buttons&CWIID_BTN_LEFT) rota(&m_izdo, &m_dcho, -1);  // con LEFT pulsado, esquiva a la izquierda
            else rota(&m_izdo, &m_dcho, 1);  // en caso contrario a la derecha
            gpioSleep(PI_TIME_RELATIVE, 0, 500000);
            fastStopMotor(&m_izdo);
            fastStopMotor(&m_dcho);
       }
       
       oledBigMessage(0, NULL);
       // Hemos esquivado el obstáculo, ahora velocidad normal si A está pulsada o coche es autonomo
       if (!mando.wiimote || mando.buttons&CWIID_BTN_A) {
            ajustaMotor(&m_izdo, velocidadCoche, ADELANTE);
            ajustaMotor(&m_dcho, velocidadCoche, ADELANTE);
       }
   }
   
   
}




