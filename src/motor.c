/**********************************************************************************
Cödigo fuente del proyecto de coche robótico
Fichero: motor.c
Fecha: 21/2/2017

Realiza el control principal del coche. Este fichero contiene el control de los motores, 
el control del mando a distancia, el control del sonar, y la lógica principal.
El control lo realizan varios threads que se comunican mediante variables atómicas.


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
#include <sys/prctl.h>
#include <stdatomic.h>
 
#include <pigpio.h>
#include <cwiid.h>
#include "imu.h"
#include "oled96.h"
#include "sound.h"
#include "pcf8591.h"
#include "bmp280.h"
#include "robot.h"

extern char *optarg;
extern int optind, opterr, optopt;

/************ Define pin numbers (Broadcom pin number scheme) *********************/
#define MI_ENA_PIN 17
#define MI_IN1_PIN 27
#define MI_IN2_PIN 22

#define MD_ENA_PIN 16
#define MD_IN1_PIN 20
#define MD_IN2_PIN 21

#define SONAR_TRIGGER_PIN 23
#define SONAR_ECHO_PIN    24

#define PITO_PIN   26
#define WMSCAN_PIN 12
#define AUDR_PIN   18
#define AUDR_ALT PI_ALT5   /* ALT function for audio pin, ALT5 for pin 18 */
#define AMPLI_PIN  25
#define LSENSOR_PIN 6
#define RSENSOR_PIN 5
#define KARR_PIN    4



/***************** Define constants and parameters ****************/
#define DISTMIN 45        /* distancia en cm a la que entendemos que hay un obstáculo */
#define INITIAL_SPEED 50  /* Entre 0 y 100% */
#define SONARDELAY 50     /* Time in ms between sonar triggers */
#define NUMPOS 3          /* Número de medidas de posición del sonar para promediar */
#define NUMPULSES 1920    /* Motor assumed is a DFRobot FIT0450 with encoder. 16 pulses per round, 1:120 gearbox */
#define WHEELD 68         /* Wheel diameter in mm */
#define KARRDELAY 150     /* Time in ms to wait between leds in KARR scan */


/***************** I2C bus addresses ****************/
#define PCF8591_I2C 0x48           /* Dirección i2c del PCF8591 (AD/DA converter) */
#define BMP280_I2C 0x76            /* Dirección i2c del BMP280 (sensor temperatura/presión) */
#define DISPLAY_I2C 0x3C           /* Dirección i2c del display SSD1306 */
#define LSM9DS1_GYR_ACEL_I2C 0x6B  /* Dirección i2c del módulo acelerómetro/giroscopio del IMU LSM9DS1 */
#define LSM9DS1_MAG_I2C 0x1E       /* Dirección i2c del módulo magnetómetro del IMU LSM9DS1 */



/****************** Variables y tipos globales **************************/

typedef enum {ADELANTE, ATRAS} Sentido_t;
typedef enum {CW, CCW} Rotation_t;

typedef struct {
    const unsigned int en_pin, in1_pin, in2_pin, sensor_pin;  /* Pines  BCM */
    Sentido_t sentido;   /* ADELANTE, ATRAS */
    int velocidad;       /* 0 a 100, velocidad (no real) objetivo impuesta al motor */
    int PWMduty;         /* Valor de PWM para alcanzar la velocidad objetivo, 0-100; no need for atomic type */
    int rpm;             /* RPM of motor, only valid if encoder is used */
    _Atomic uint32_t counter;  /* counter for encoder pulses */
    uint32_t speedsetTick;     /* system tick value when the variable velocidad is set */
    pthread_mutex_t mutex;     /* Mutex to avoid collision when several threads access motor */
} Motor_t;


typedef struct {
    _Atomic cwiid_wiimote_t *wiimote;
    _Atomic uint16_t buttons;
} MandoWii_t;


typedef struct {
    const unsigned int pin;
    bool pitando;
    pthread_mutex_t mutex;
} Bocina_t;


typedef struct {
    const unsigned int trigger_pin, echo_pin;
    _Atomic bool triggered;
    uint32_t distance;
} SonarHCSR04_t;


/* Timers used for periodic tasks (threads) using the pigpio library function gpioSetTimerFunc */
enum timers {TIMER0, TIMER1, TIMER2, TIMER3, TIMER4, TIMER5, TIMER6, TIMER7, TIMER8, TIMER9};

/** These are the shared memory variables used for thread intercommunication **/
_Atomic uint32_t distance = UINT32_MAX;
_Atomic int velocidadCoche = INITIAL_SPEED;  // velocidad objetivo del coche. Entre 0 y 100; el sentido de la marcha viene dado por el botón pulsado (A/B)
_Atomic bool esquivando; // Car is avoiding obstacle
_Atomic bool stalled;    // Car is stalled: it does not change its distance to objects
_Atomic bool collision;  // Car has crashed, when moving forwards or backwards
_Atomic bool scanningWiimote;  // User pressed scan button and car is scanning for wiimotes
_Atomic bool playing_audio, cancel_audio;   // Variables compartidas con fichero sound.c

/* Generic global variables */
int soundVolume = 96;  // 0 - 100%
sem_t semaphore;  // Used to synchronize the main loop with the sonar measurement thread
bool remoteOnly, useEncoder, checkBattery, softTurn, calibrateIMU; // program line options
char *alarmFile = "sounds/police.wav";  // File to play when user presses "UP" in wiimote


/********** Define and initialize the objects which compose the robotic car *********/
MandoWii_t mando;

Bocina_t bocina = {
    .pitando = false,
    .pin = PITO_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};

SonarHCSR04_t sonarHCSR04 = {
    .trigger_pin = SONAR_TRIGGER_PIN,
    .echo_pin = SONAR_ECHO_PIN,
    .distance = UINT32_MAX,
    .triggered = false
};

Motor_t m_izdo = {
    .en_pin = MI_ENA_PIN,
    .in1_pin = MI_IN1_PIN,
    .in2_pin = MI_IN2_PIN,
    .sensor_pin = LSENSOR_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};

Motor_t m_dcho = {
    .en_pin = MD_ENA_PIN,
    .in1_pin = MD_IN1_PIN,
    .in2_pin = MD_IN2_PIN,
    .sensor_pin = RSENSOR_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};



/* Forward declarations of internal functions of this module */
void speedControl(void);  /* Callback called periodically to make motors rotate at same RPM */
void ajustaCocheConMando(void); /* Read wiimote buttons and adjust speed accordingly */


/* Callbacks called when some GPIO pin changes */
void speedSensor(int gpio, int level, uint32_t tick);
void wmScan(int gpio, int level, uint32_t tick);



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
    motor->speedsetTick = gpioTick();
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
    motor->speedsetTick = gpioTick();
    pthread_mutex_unlock(&motor->mutex);   
}



int setupMotor(Motor_t *motor)
{
    int r = 0;
    
    r |= gpioSetMode(motor->in1_pin, PI_OUTPUT);
    r |= gpioSetMode(motor->in2_pin, PI_OUTPUT);
    
    if (gpioSetPWMfrequency(motor->en_pin, 500)<0) r = -1;   /* 500 Hz, low but not very audible */
    if (gpioSetPWMrange(motor->en_pin, 100)<0) r = -1;       /* Range: 0-100, real range = 2000 */
    
    if (useEncoder) {
        r |= gpioSetMode(motor->sensor_pin, PI_INPUT);
        gpioSetAlertFunc(motor->sensor_pin, speedSensor); 
        gpioSetTimerFunc(TIMER2, 100, speedControl);  // Control velocidad motores cada 100 ms, timer#2         
    }
    
    if (r) fprintf(stderr, "Cannot initialise motor!\n");
    return r;
}


void closeMotor(Motor_t *motor)
{
   printf("Closing motor...\n");
   if (useEncoder) {
      gpioSetAlertFunc(motor->sensor_pin, NULL); 
      gpioSetTimerFunc(TIMER2, 100, NULL);
   }
   fastStopMotor(motor);
}



/****************** Funciones de control del sensor de distancia de ultrasonidos HC-SR04 **************************/

/* trigger a sonar reading every SONARDELAY milliseconds */
void sonarTrigger(void)
{
   gpioWrite(sonarHCSR04.trigger_pin, PI_ON);
   gpioDelay(10);     /* 10us trigger pulse */
   gpioWrite(sonarHCSR04.trigger_pin, PI_OFF);
   atomic_store_explicit(&sonarHCSR04.triggered, true, memory_order_relaxed); // sonarHCSR04.triggered = true;
}


/* callback called when the SONAR_ECHO pin changes state.
   It sets global variables "distance" and "stalled", as the only producer of these variables.
   It sets global variable "esquivando", main loop also sets this variable  */
void sonarEcho(int gpio, int level, uint32_t tick)
{
static uint32_t startTick, referenceTick, distance_array[NUMPOS], pos_array;
static uint32_t previous_distance, reference_distance;
static int firstTime;
static bool false_echo;
static const int maxStalledTime = 1200*1e3;  // Time in microseconds to flag car as stopped (it does not change its distance)
static const char displayText[] = "Dist (cm):";
uint32_t suma, distance_local;
char str[6];
int i, diffTick, stalledTime=0;
bool in_collision, is_stalled;
  
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

           distance_array[pos_array++] = (diffTick*17)/1000;  // sonar measured distance in cm
           if (pos_array == NUMPOS) pos_array = 0;
 
           if (firstTime>=0) {
              if (firstTime < NUMPOS) { /* The first NUMPOS times until array is filled */
                  firstTime++;
                  break;
              } else {
                 firstTime = -1;  /* Initialisation of distance_array is over */
                 referenceTick = tick;  // Reference for stalled time calculation
                 oledWriteString(0, 0, displayText, false);  // Write fixed text to display only once
              }
           }
           
           /* Calculate moving average */
           for (i=0, suma=0; i<NUMPOS; i++) suma += distance_array[i]; 
           sonarHCSR04.distance = suma/NUMPOS;   
           
           /* Set global variable "distance", this is the only producer */
           atomic_store_explicit(&distance, sonarHCSR04.distance, memory_order_release);
           distance_local = sonarHCSR04.distance; // local copy of variable
           if (referenceTick == tick) reference_distance = distance_local;  // Will only happen once, at the beginning
           
           /* Update display if distance changed since last reading */
           if (distance_local != previous_distance) {
               snprintf(str, sizeof(str), "%-3u", distance_local);
               oledWriteString(8*sizeof(displayText), 0, str, false);  // update only distance number
               previous_distance = distance_local;
           }
           
           /* If car should be moving, look at change in distance to object since reference was taken; 
              if distance change is small, compute time passed as stalled, otherwise, reset values */
           if ((m_izdo.velocidad || m_dcho.velocidad) && abs(reference_distance - distance_local)<=2) stalledTime = tick - referenceTick;
           else {
               stalledTime = 0;
               referenceTick = tick;
               reference_distance = distance_local;
           }
           
           /* If the stalled time is above threshold, set global variable "stalled" as true, otherwise as false */
           is_stalled = stalledTime >= maxStalledTime;
           atomic_store_explicit(&stalled, is_stalled, memory_order_release); 
           
           /* Activate semaphore to indicate main loop that it must awake;
              check specific situations first, and then activate semaphore if one of two conditions is met:
              either the distance to obstacle is below threshold or the car is stalled (below or over threshold)*/
           if (!remoteOnly && !esquivando && !scanningWiimote) {
               in_collision = atomic_load_explicit(&collision, memory_order_acquire);
               if (distance_local < DISTMIN || is_stalled || in_collision) {
                  atomic_store_explicit(&esquivando, true, memory_order_release);  // Set global variable
                  i = sem_post(&semaphore); // Awake main loop
                  if (i) perror("Error when activating semaphore");
               }
           }
           
           break;
   } 
   // Only executed after PI_OFF
   atomic_store_explicit(&sonarHCSR04.triggered, false, memory_order_relaxed); 
}


int setupSonarHCSR04(void)
{
   gpioSetMode(sonarHCSR04.trigger_pin, PI_OUTPUT);
   gpioWrite(sonarHCSR04.trigger_pin, PI_OFF);
   gpioSetMode(sonarHCSR04.echo_pin, PI_INPUT);

   /* update sonar several times a second */
   if (gpioSetTimerFunc(TIMER0, SONARDELAY, sonarTrigger) ||     /* trigger sonar, timer#0 */
        gpioSetAlertFunc(sonarHCSR04.echo_pin, sonarEcho)) {     /* monitor sonar echos */
        fprintf(stderr, "Error al inicializar el sonar!\n");
        return -1;
       }
   return 0;
}


void closeSonarHCSR04(void)
{
   printf("Closing sonar...\n");
   gpioSetTimerFunc(TIMER0, SONARDELAY, NULL);
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


/* Reproduce "file" en otro hilo si no se está reproduciendo nada. Si es el caso, cancela reproducción y vuelve.
file debe ser un string invariable, en memoria 
modo=0; vuelve inmediatamente, sin esperar el final
modo=1; vuelve después de haber reproducido el fichero de audio */
void audioplay(char *file, int modo)
{
pthread_t pth;
    
    /* Si ya estamos reproduciendo algo, manda señal de cancelación al thread de audio */
    if (playing_audio) {
        atomic_store_explicit(&cancel_audio, true, memory_order_relaxed);  // Signal cancel to sound thread
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
int i;
static uint16_t previous_buttons;
unsigned int bateria;
static int LEDs[4] = { CWIID_LED1_ON,  CWIID_LED1_ON | CWIID_LED2_ON,
                       CWIID_LED1_ON | CWIID_LED2_ON | CWIID_LED3_ON,
                       CWIID_LED1_ON | CWIID_LED2_ON | CWIID_LED3_ON | CWIID_LED4_ON };        

     for (i = 0; i < mesg_count; i++) {
        switch (mesg[i].type) {
        case CWIID_MESG_BTN:  // Change in buttons
            atomic_store_explicit(&mando.buttons, mesg[i].btn_mesg.buttons, memory_order_release);

            // Save data from accelerometer in a file. Start and end saving when '2' is pressed
            if (previous_buttons&CWIID_BTN_2 && ~mando.buttons&CWIID_BTN_2) save_accel_data();

            /*** Botones + y - ***/           
            if (previous_buttons&CWIID_BTN_PLUS && ~mando.buttons&CWIID_BTN_PLUS) {
                if (mando.buttons&CWIID_BTN_1) { /* sube volumen: buttons ´1´ + ´+´ */
                    soundVolume += 2;
                    if (soundVolume > 100) soundVolume = 100;
                    setVolume(soundVolume);
                } else {  /* ajusta la velocidad del coche y la marca en leds del mando */
                    atomic_fetch_add_explicit(&velocidadCoche, 10, memory_order_relaxed);
                    if (velocidadCoche > 100) atomic_store_explicit(&velocidadCoche, 100, memory_order_release);
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
                    atomic_fetch_sub_explicit(&velocidadCoche, 10, memory_order_relaxed);
                    if (velocidadCoche < 0) atomic_store_explicit(&velocidadCoche, 0, memory_order_release);
                    cwiid_set_led(wiimote, LEDs[velocidadCoche/26]);
                }
            }
            
            /*** Botones A, B y RIGHT, LEFT; si estamos esquivando, no: el loop de main tiene el control ***/
            if (!esquivando) ajustaCocheConMando();
    
            /*** pito ***/
            if (~previous_buttons&CWIID_BTN_DOWN && mando.buttons&CWIID_BTN_DOWN) activaPito();    
            if (previous_buttons&CWIID_BTN_DOWN && ~mando.buttons&CWIID_BTN_DOWN) desactivaPito();    
            
            
            /*** sonido ***/
            if (~previous_buttons&CWIID_BTN_UP && mando.buttons&CWIID_BTN_UP) audioplay(alarmFile, 0);
        
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



void closeWiimote(void)
{
    printf("Closing wiimote...\n");
    if (mando.wiimote) cwiid_close((cwiid_wiimote_t*)mando.wiimote);
    atomic_store_explicit(&mando.wiimote, NULL, memory_order_release);
    atomic_store_explicit(&mando.buttons, 0, memory_order_release);       
}
      
     

void setupWiimote(void)
{
static uint8_t bluetooth_glyph[] = {0, 66, 36, 255, 153, 90, 36, 0};
cwiid_wiimote_t *wiimote;
bdaddr_t ba;
    
    closeWiimote();
    cwiid_set_err(wiiErr);
    oledSetBitmap8x8(15*8, 0, NULL);  // 15: last position in line (0-15), clear BT icon
    oledBigMessage(0, "Scan... ");
    pito(5, 1);   // Pita 5 décimas para avisar que comienza búsqueda de mando
    
    printf("Pulsa las teclas 1 y 2 en el mando de la Wii...\n");
    gpioSleep(PI_TIME_RELATIVE, 2, 0);  // para dar tiempo a desconectar el mando si estaba conectado
    ba = *BDADDR_ANY;
    wiimote = cwiid_open_timeout(&ba, 0, 5);  // 5 seconds timeout
    oledBigMessage(0, NULL);
    if (!wiimote ||
        cwiid_set_rpt_mode(wiimote, CWIID_RPT_BTN | CWIID_RPT_STATUS) || 
        cwiid_set_mesg_callback(wiimote, wiiCallback) ||
        cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC)) {
            fprintf(stderr, "No puedo conectarme al mando de la Wii!\n");
            return;  // No es error si no hay wiimote, el coche funciona sin mando
    } 
    
    // wiimote found
    atomic_store_explicit(&mando.wiimote, (_Atomic cwiid_wiimote_t*)wiimote, memory_order_release);
    printf("Conectado al mando de la Wii\n");
    oledSetBitmap8x8(15*8, 0, bluetooth_glyph);  // Put BT icon
    cwiid_set_rumble(wiimote, 1);  // señala mediante zumbido el mando sincronizado
    gpioSleep(PI_TIME_RELATIVE, 0, 500000);   // Espera 0,5 segundos
    cwiid_set_rumble(wiimote, 0);
    return;
}

 
   
static void* scanWiimotes(void *arg)
{
    atomic_store_explicit(&scanningWiimote, true, memory_order_release); // signal that scanning is in place
    fastStopMotor(&m_izdo); fastStopMotor(&m_dcho);   // Para el coche mientras escanea wiimotes 
    atomic_store_explicit(&velocidadCoche, 0, memory_order_release);

    oledWriteString(12*8, 1, "    ", false); // Borra mensaje de "Auto", si está           
    setupWiimote(); 
    atomic_store_explicit(&velocidadCoche, INITIAL_SPEED, memory_order_release); // Nueva velocidad inicial, con o sin mando 
 
    if (!mando.wiimote && !remoteOnly) {  // No hay mando, coche es autónomo
        oledWriteString(12*8, 1, "Auto", false);
        ajustaMotor(&m_izdo, velocidadCoche, ADELANTE);
        ajustaMotor(&m_dcho, velocidadCoche, ADELANTE);                
    } 
    atomic_store_explicit(&scanningWiimote, false, memory_order_release);  // signal that scanning is over
    return NULL;
}



/* callback llamado cuando el pin WMSCAN_PIN cambia de estado. Tiene un pull-up a VCC, OFF==pulsado */
void wmScan(int gpio, int level, uint32_t tick)
{
static uint32_t pressTime;
static bool pressed;
static pthread_t pth;
    
    switch (level) {
        case PI_ON:   // Sync button released
            if (!pressed) return;   // elimina clicks espureos
            pressed = false;
            
            /* Long press (>2 sec): shutdown */
            if (tick-pressTime > 2*1e6) {
                oledBigMessage(0, "MANUAL");
                oledBigMessage(1, "SHUTDOWN");
                pito(10, 1);
                closedown();
                execlp("sudo", "sudo", "poweroff", NULL);  
                exit(1); 
            }
            
            /* Short press: scan wiimotes in another thread, in order to return quickly to caller */           
            if (!scanningWiimote && !pthread_create(&pth, NULL, scanWiimotes, NULL)) pthread_detach(pth);    
            break;
            
        case PI_OFF:  // Sync button pressed
            pressed = true;
            pressTime = tick;
            break;
    }
}


/* Read wiimote buttons and adjust speed accordingly */
void ajustaCocheConMando(void)
{
int v_izdo, v_dcho;
Sentido_t s_izdo, s_dcho;

   v_izdo = v_dcho = 0;
   s_izdo = s_dcho = ADELANTE;
   
   /*** Botones A y B, leen la variable global "velocidadCoche" ***/
   if (mando.wiimote && mando.buttons&(CWIID_BTN_A | CWIID_BTN_B)) { // if A or B or both pressed
      v_izdo = v_dcho = velocidadCoche;
      if (mando.buttons&CWIID_BTN_A) s_izdo = s_dcho = ADELANTE;
      else s_izdo = s_dcho = ATRAS;  // si vamos marcha atrás (botón B), invierte sentido
    
      /*** Botones LEFT y RIGHT, giran el coche ***/
      if (mando.buttons&CWIID_BTN_RIGHT) {
         s_dcho = 1 - s_dcho;  // Invert direction of movement
         if (softTurn) v_dcho = 0;
         else v_dcho = 50;
         v_izdo += 10; 
      } 
            
      if (mando.buttons&CWIID_BTN_LEFT) {
         s_izdo = 1 - s_izdo;  // Invert direction of movement
         if (softTurn) v_izdo = 0; 
         else v_izdo = 50;
         v_dcho += 10;                
      }
   }
   
   /*** Ahora activa la velocidadCoche calculada en cada motor ***/
   ajustaMotor(&m_izdo, v_izdo, s_izdo);
   ajustaMotor(&m_dcho, v_dcho, s_dcho);           
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
        case PI_OFF:
            // Increment counter by one in an atomic way (other threads must see correct value)
            atomic_fetch_add_explicit(&motor->counter, 1, memory_order_relaxed); //  motor->counter++;
            break;           
    }    
}


/* Callback llamado regularmente. Realiza el lazo de control de la velocidad, comparando
la diferencia de velocidades entre los motores para igualarlas */
void speedControl(void)
{
static uint32_t past_tick; 
uint32_t current_tick;
int period;
   
static uint32_t past_lcounter;  
uint32_t current_lcounter;  
int lpulses, lfreq, lpwm;

static uint32_t past_rcounter;    
uint32_t current_rcounter;  
int rpulses, rfreq, rpwm;
  
int pv, kp=1;  // parameters of PID filter: pv is measured error (process value), kp is proportionality constant
static const int minMotorSetupTime = 1E6;  // Allow for 1 second (1E6 microseconds) for motors to stabilise before PID control loop works
  
static FILE *fp;  
  
    current_tick = gpioTick();
    current_lcounter =  m_izdo.counter;
    current_rcounter =  m_dcho.counter;
    if (past_tick == 0) {    // First time speedControl gets called
       past_tick = current_tick;
       //fp = fopen("motors.txt", "w");
       if (fp) setlinebuf(fp);
       if (fp) fprintf(fp, "ticks,m_izdo.rpm,m_dcho.rpm,m_izdo.PWMduty,m_dcho.PWMduty,m_izdo.velocidad,m_dcho.velocidad\r\n");
       return;
    }
    period = current_tick - past_tick;
    past_tick = current_tick;
        
    /***** Left motor *****/
    lpulses = current_lcounter - past_lcounter;
    lfreq = (1E6*lpulses)/period;  
    m_izdo.rpm = lfreq*60/NUMPULSES;
    past_lcounter = current_lcounter;
    //printf("Left motor: pulses=%d, freq=%d, rpm=%d\n", lpulses, lfreq, m_izdo.rpm);
    
    /***** Right motor *****/
    rpulses = current_rcounter - past_rcounter;
    rfreq = (1E6*rpulses)/period;  
    m_dcho.rpm = rfreq*60/NUMPULSES;
    past_rcounter = current_rcounter;
    //printf("Right motor: pulses=%d, freq=%d, rpm=%d\n", rpulses, rfreq, m_dcho.rpm);
    
    if (fp) fprintf(fp, "%u,%d,%d,%d,%d,%d,%d\r\n", current_tick, m_izdo.rpm, m_dcho.rpm, m_izdo.PWMduty, m_dcho.PWMduty, m_izdo.velocidad, m_dcho.velocidad);
    
    /******* P control loop. SP=0, PV=lpulses-rpulses *********/
    if (m_izdo.velocidad != m_dcho.velocidad) return;  // Enter control section only if straight line desired: both speeds equal
    if (m_izdo.velocidad == 0) return;  // If speed is 0 (in both), do not enter control section
    /*** If time elapsed since speed was set in motor is below a threshold, 
         so that it had no time to stabilise, do not enter ***/ 
    if (current_tick - m_izdo.speedsetTick < minMotorSetupTime) return;
    if (current_tick - m_dcho.speedsetTick < minMotorSetupTime) return;
    
    pv = lpulses - rpulses;
    if (abs(pv) <= 8) return;  // Tolerable error, do not enter control section
    
    /** Control section loop; adjust parameters **/
    //printf("\tAdjust left: %i, right: %i\n", -(kp*pv)/10, (kp*pv)/10);
    pthread_mutex_lock(&m_izdo.mutex);
    pthread_mutex_lock(&m_dcho.mutex);   
    m_izdo.PWMduty -= (kp*pv)/10;  // PWMduty can go outside the interval [0,100], let it go
    m_dcho.PWMduty += (kp*pv)/10;
    lpwm = m_izdo.PWMduty; rpwm = m_dcho.PWMduty;  // But lpwm and rpwm cannot go out of [0,100]
    if (lpwm>100) lpwm = 100; if (lpwm<0) lpwm = 0;
    if (rpwm>100) rpwm = 100; if (rpwm<0) rpwm = 0;
    gpioPWM(m_izdo.en_pin, lpwm);
    gpioPWM(m_dcho.en_pin, rpwm);   
    pthread_mutex_unlock(&m_izdo.mutex); 
    pthread_mutex_unlock(&m_dcho.mutex);
}




/****************** Funciones auxiliares varias **************************/

static int interruptibleWait(int duration)
{
int rest, lapse;
uint32_t startTick;
const int chunk = 10000;  // 10 ms chunks
bool in_collision;   
   
   startTick = gpioTick();

   while(rest = duration - (gpioTick() - startTick), rest>0) {
      lapse = (rest>chunk)?chunk:rest;
      gpioDelay(lapse);
      in_collision = atomic_load_explicit(&collision, memory_order_acquire);
      if (in_collision) return -1;
   }

   return 0;
}




/*
Rota el coche a la derecha (dextrógiro, rotation==CW) o a la izquierda (levógiro, rotation==CCW). 
Rota durante 'duration' microseconds
*/
static int rota(Rotation_t rotation, Sentido_t marcha, int duration)  
{
Motor_t *pivot, *non_pivot;

   switch (marcha) {
      case ADELANTE: if (rotation == CW) {
                        pivot = &m_dcho;
                        non_pivot = &m_izdo;
                     }
                     else {
                        pivot = &m_izdo;
                        non_pivot = &m_dcho;
                     }
                     break;
      case ATRAS: if (rotation == CW) {
                        pivot = &m_izdo;
                        non_pivot = &m_dcho;
                  }
                  else {
                     pivot = &m_dcho;
                     non_pivot = &m_izdo;
                  }
                  break;
   }

   ajustaMotor(pivot, softTurn?0:velocidadCoche, 1-marcha);
   ajustaMotor(non_pivot, velocidadCoche, marcha);
   return interruptibleWait(duration);
}



/**
The car has found an obstacle in front, move backwards and turn slightly 
**/
static int retreatBackwards(void)
{
int rc;

   //printf("Car seems stalled or collisioned, move a bit backwards...\n");
   fastStopMotor(&m_izdo); fastStopMotor(&m_dcho);
   gpioSleep(PI_TIME_RELATIVE, 0, 200000);
   ajustaMotor(&m_izdo, 50, ATRAS);
   ajustaMotor(&m_dcho, 50, ATRAS);
   rc = interruptibleWait(softTurn?400000:800000);  // Move a little backwards first
   if (rc >= 0) rc = rota(CW, ATRAS, velocidadCoche>70?300000:600000);  // If all went well, rotate backwards

   fastStopMotor(&m_izdo); fastStopMotor(&m_dcho); 
   return rc;
}



/**
Loop called when an obstable was detected by the sonar. It tries to avoid it,
and only returns if it was avoided or the user stopped pressing A.
The variable "esquivando" will be true when called, main loop will wait for this
routine to finish. So it has the only control of the car.
**/
static void avoidObstacle(void)
{
int rc;
   
   //printf("Obstacle at %d cm, avoiding...\n", distance);
   /** Loop for obstacle avoidance **/
   // distance is atomic, and is set asynchronously in another thread
   while (atomic_load_explicit(&distance, memory_order_acquire) < DISTMIN) {  
      /** Check that the button to scan the wiimote was not pressed **/
      if (scanningWiimote) break;
      /** Check that the user keeps pressing A **/
      if (mando.wiimote && ~mando.buttons&CWIID_BTN_A) break;

      /**  Rotate the car to avoid obstacle; Rotate for the time to get a new distance measure **/
      if (mando.wiimote && mando.buttons&CWIID_BTN_LEFT) rc = rota(CCW, ADELANTE, SONARDELAY*1000);  // con LEFT pulsado, esquiva a la izquierda
      else rc = rota(CW, ADELANTE, SONARDELAY*1000);  // en caso contrario a la derecha
      
      if (rc < 0 || atomic_load_explicit(&stalled, memory_order_acquire)) rc = retreatBackwards();  // stalled is a global variable, set by the sonar asynchronously
   }
}




/* Thread in charge of sending a clock pulse to the circuit implementing the KARR scan effect.
At each LH transition, the led will change */
static void karrScan(void)
{  
   gpioWrite(KARR_PIN, PI_ON); 
   gpioSleep(PI_TIME_RELATIVE, 0, 100);  // set high state for clock during 100 us 
   gpioWrite(KARR_PIN, PI_OFF);
}



/* Activa el efecto scanner de los leds delanteros, como KARR */
static void setupKarr(void)
{    
   gpioSetMode(KARR_PIN, PI_OUTPUT);
   gpioWrite(KARR_PIN, PI_OFF); 
   gpioSetTimerFunc(TIMER5, KARRDELAY, karrScan);    // Cool KARR LED effect, timer#5
}


static void closeKarr(void)
{
   gpioSetTimerFunc(TIMER5, KARRDELAY, NULL); 
}


/* Terminate program, clean up, restore settings so that car stops */
void closedown(void)
{
   printf("\n");
   closeSonarHCSR04();
   closeWiimote();
   closeMotor(&m_izdo);
   closeMotor(&m_dcho);
   closeSound();
   closeLSM9DS1();
   closeBMP280();
   closeKarr();
   gpioSetPullUpDown(WMSCAN_PIN, PI_PUD_OFF);
   gpioWrite(bocina.pin, PI_OFF);
   gpioSetMode(bocina.pin, PI_INPUT);
   gpioSetMode(KARR_PIN, PI_INPUT);
}



/* Para el coche, cierra todo y termina el programa */
void terminate(int signum)
{
   closedown();
   if (checkBattery) closePCF8591();  // Do not call this in closedown()
   oledShutdown();
   gpioTerminate();
   printf("Bye\n");
   exit(1);
}



int setup(void)
{
int rc = 0;
   
   // Initialise pigpio library
   if (gpioCfgClock(5, PI_CLOCK_PCM, 0)<0) return 1;   /* Standard settings: Sample rate: 5 us, PCM clock */
   gpioCfgInterfaces(PI_DISABLE_FIFO_IF | PI_DISABLE_SOCK_IF);
   if (gpioInitialise()<0) return 1;
   if (gpioSetSignalFunc(SIGINT, terminate)<0) return 1;  // Call ´terminate´ when Ctrl-C is pressed
   
   // Restore signal actions to default, so program dumps core if they happen
   signal(SIGSEGV, SIG_DFL);  
   signal(SIGFPE, SIG_DFL); 
   signal(SIGILL, SIG_DFL);
   signal(SIGBUS, SIG_DFL);
   
   if (setuid(getuid()) < 0) return 1;  // Drop root privileges if setuid
   prctl(PR_SET_DUMPABLE, 1, 0, 0, 0);  // Set program to dump core if it crashes
    
   // Inicializa display
   oledInit(DISPLAY_I2C);
   oledSetInversion(true);   // Fill display, as life sign
      
   // Inicializa altavoz y bocina
   setupSound(AMPLI_PIN);
   setVolume(soundVolume);
   gpioSetMode(AUDR_PIN, AUDR_ALT);  // Saca PWM0 (audio right) por el GPIO al amplificador
   gpioSetMode(bocina.pin, PI_OUTPUT);
   gpioWrite(bocina.pin, PI_OFF);
   
   if (checkBattery) rc |= setupPCF8591(PCF8591_I2C, TIMER1);  // Check battery voltage, timer#1

   gpioSetMode(WMSCAN_PIN, PI_INPUT);
   gpioSetPullUpDown(WMSCAN_PIN, PI_PUD_UP);  // pull-up resistor; button pressed == OFF
   gpioGlitchFilter(WMSCAN_PIN, 100000);      // 0,1 sec filter
   
   rc |= setupMotor(&m_izdo);
   rc |= setupMotor(&m_dcho);
   rc |= sem_init(&semaphore, 0, 0);
   
   setupBMP280(BMP280_I2C, TIMER4);  // Setup temperature/pressure sensor
   setupLSM9DS1(LSM9DS1_GYR_ACEL_I2C, LSM9DS1_MAG_I2C, calibrateIMU, TIMER3);   // Setup IMU
   
   setupWiimote(); 
   gpioSetAlertFunc(WMSCAN_PIN, wmScan);  // Call wmScan when button changes. Debe llamarse después de setupWiimote
   
   setupKarr();  // start KARR scanner effect
   oledSetInversion(false); // clear display
   
   rc |= setupSonarHCSR04();  // last to call, as it starts measuring distance and triggering semaphore

   return rc;
}


/****************** Main **************************/
void main(int argc, char *argv[])
{
int rc;
double volts;
bool is_stalled, in_collision;
uint32_t distance_value;

   opterr = 0;  // Prevent getopt from outputting error messages
   while ((rc = getopt(argc, argv, "crbesf:")) != -1)
       switch (rc) {
           case 'r':  /* Remote only mode: only reacts to remote control */
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
   
   rc = setup();
   if (rc) {
       fprintf(stderr, "Error al inicializar. Coche no arranca!\n");
       if (rc < 0) terminate(SIGINT);   // Si el error estaba al inicializar pigpio (rc>0), no llames a terminate
       else exit(1);
   }
  
   oledBigMessage(0, " Ready  ");
   audioplay("sounds/ready.wav", 1);
   oledBigMessage(0, NULL);
   
   // Check if battery low; -1 means that the ADC does not work correctly
   volts = getMainVoltageValue();  
   if (checkBattery && volts>=0 && volts<6.6) {
      oledBigMessage(0, "Bateria!");
      audioplay("sounds/batterylow.wav", 1);
      oledBigMessage(0, NULL);           
   }

   if (!mando.wiimote && !remoteOnly) {  // No hay mando, el coche es autónomo
        oledWriteString(12*8, 1, "Auto", false);    
   }
   
   
   /*** Main control loop ***/
   for (;;) {
       /* Signal to sonarEcho that the semaphore can be activated: car is not 'esquivando' */
       atomic_store_explicit(&esquivando, false, memory_order_release); 
       
       /* Adjust car to move */
       if (mando.wiimote || remoteOnly) ajustaCocheConMando();  // wiimote controlled car
       else {  // autonomous car
         ajustaMotor(&m_izdo, velocidadCoche, ADELANTE);
         ajustaMotor(&m_dcho, velocidadCoche, ADELANTE);             
       }
         
       /* Sleep until semaphore awakens us; it will happen in 3 cases:
          either the distance to an obstacle is below the threshold, or the car is stalled, or there was a collision */
       rc = sem_wait(&semaphore);  
       if (rc) {
         perror("Error waiting for semaphore");
         continue;
       }
             
       if (mando.wiimote && ~mando.buttons&CWIID_BTN_A) continue;  // Take action only if A is pressed
       distance_value = atomic_load_explicit(&distance, memory_order_acquire); 
       in_collision = atomic_load_explicit(&collision, memory_order_acquire);   
       is_stalled = atomic_load_explicit(&stalled, memory_order_acquire);   
     
       oledBigMessage(0, "OBSTACLE"); 
       if (in_collision || is_stalled) {
          printf("Collision!\n");
          retreatBackwards(); 
       }
       else if (distance_value < DISTMIN) avoidObstacle();  // Distance is below threshold       

       /* Obstacle is avoided, go back to normality */
       oledBigMessage(0, NULL);
   }
}




