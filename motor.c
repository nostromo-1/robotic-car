/**********************************************************************************
Cödigo fuente del proyecto de coche robótico
Fichero: motor.c
Fecha: 21/2/2017

Realiza el control principal del coche


***********************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <malloc.h>
#include <string.h>
  
#include <pigpio.h>
#include <cwiid.h>


#define MI_ENA 17
#define MI_IN1 27
#define MI_IN2 22

#define MD_ENA 16
#define MD_IN1 20
#define MD_IN2 21

#define SONAR_TRIGGER 23
#define SONAR_ECHO    24
#define NUMPOS 2    /* Número de medidas de posición promediadas en distancia */

#define PITO_PIN 26
#define VBAT_PIN 19
#define WMSCAN_PIN 12
#define AUDR_PIN 18
#define AUDR_ALT PI_ALT5   /* ALT function for audio pin, ALT5 for pin 18 */

#define DISTMIN 50  /* distancia a la que entendemos que hay un obstáculo */


void* play_wav(void *filename);  // Función en fichero sound.c
volatile int playing_audio, cancel_audio;  // Variables compartidas con fichero sound.c

 
/****************** Variables y tipos globales **************************/

typedef enum {ADELANTE=0, ATRAS=1} Sentido_t;
typedef struct {
    const uint en_pin, in1_pin, in2_pin;  /* Pines  BCM */
    Sentido_t sentido;   /* ADELANTE, ATRAS */
    int velocidad;         /* -100 a 100 */
} Motor_t;


typedef struct {
    cwiid_wiimote_t *wiimote;
    uint16_t buttons;
} MandoWii_t;


typedef struct {
    int pitando;
    const uint pin;
    pthread_mutex_t mutex;
} Bocina_t;


volatile uint32_t distancia = UINT32_MAX;
volatile int velocidad = 50;  // velocidad objetivo del coche. Entre 0 y 100; el sentido de la marcha viene dado por el botón pulsado (A/B)
volatile int powerState = PI_OFF;
volatile int esquivando;
sem_t semaphore;


MandoWii_t mando;

Bocina_t bocina = {
    .pitando = 0,
    .pin = PITO_PIN,
    .mutex = PTHREAD_MUTEX_INITIALIZER
};

Motor_t m_izdo = {
    .en_pin = MI_ENA,
    .in1_pin = MI_IN1,
    .in2_pin = MI_IN2,
    .sentido = ADELANTE,
    .velocidad = 0
};

Motor_t m_dcho = {
    .en_pin = MD_ENA,
    .in1_pin = MD_IN1,
    .in2_pin = MD_IN2,
    .sentido = ADELANTE,
    .velocidad = 0
};





/****************** Funciones de control de los motores **************************/
static void ajustaSentido(Motor_t *motor, Sentido_t dir)
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


void stopMotor(Motor_t *motor)
{
    gpioWrite(motor->in1_pin, PI_OFF);
    gpioWrite(motor->in2_pin, PI_OFF);
    motor->velocidad = 0;
}


/* v va de -100 a 100 */
void ajustaVelocidad(Motor_t *motor, int v)
{    
    if (v > 100) v = 100;
    if (v < -100) v = -100;
    if (motor->velocidad == v) return;
    motor->velocidad = v;
    if (v>0) {  // ADELANTE
        ajustaSentido(motor, ADELANTE);
    }
    else {  //ATRAS
        ajustaSentido(motor, ATRAS);
        v = -v;
    }
    gpioPWM(motor->en_pin, v);
}



int setupMotor(Motor_t *motor)
{
    int r = 0;
    
    r |= gpioSetMode(motor->in1_pin, PI_OUTPUT);
    r |= gpioSetMode(motor->in2_pin, PI_OUTPUT);
    
    if (gpioSetPWMfrequency(motor->en_pin, 100)<0) r = -1;   /* 100 Hz */
    if (gpioSetPWMrange(motor->en_pin, 100)<0) r = -1;       /* Máximo: 100%, real range = 2000 */
    if (r) fprintf(stderr, "Cannot initialise motor!\n");
    return r;
}



/****************** Funciones de control del sensor de distancia de ultrasonidos HC-SR04 **************************/

/* trigger a sonar reading */
void sonarTrigger(void)
{
   gpioWrite(SONAR_TRIGGER, PI_ON);
   gpioDelay(10); /* 10us trigger pulse */
   gpioWrite(SONAR_TRIGGER, PI_OFF);
}


/* callback llamado cuando el pin SONAR_ECHO cambia de estado. Ajusta la variable global distancia */
void sonarEcho(int gpio, int level, uint32_t tick)
{
   static uint32_t startTick, endTick;
   static uint32_t distance_array[NUMPOS], pos_array=0;
   uint32_t diffTick, d;
   uint32_t i, suma;
   static int firstTime=0;

 switch (level) {
 case PI_ON: 
           startTick = tick;
           break;

 case PI_OFF: 
           endTick = tick;
           diffTick = endTick - startTick;
           if (diffTick > 30000 || diffTick < 50) break;  /* out of range */
           d = (diffTick*17)/1000;

           distance_array[pos_array++] = d;
           if (pos_array == NUMPOS) pos_array = 0;
 
           if (firstTime>=0) {
              if (firstTime < NUMPOS) { /* The first NUMPOS times until array is filled */
                  distancia = d;
                  firstTime++;
                  break;
              }
              else firstTime = -1;  /* Initialisation is over */
           }
           
           /* Calculate moving average */
           for (i=0, suma=0; i<NUMPOS; i++) suma += distance_array[i]; 
            distancia = suma/NUMPOS;   /* La variable de salida, global */
           if (distancia < DISTMIN && esquivando == 0) {
                esquivando = 1;
                i = sem_post(&semaphore);
                if (i) perror("Error al activar semáforo");
           }
              break;
 } 
}


int setupSonar(void)
{
   gpioSetMode(SONAR_TRIGGER, PI_OUTPUT);
   gpioWrite(SONAR_TRIGGER, PI_OFF);
   gpioSetMode(SONAR_ECHO, PI_INPUT);

   /* update sonar 20 times a second, timer #0 */
   if (gpioSetTimerFunc(0, 50, sonarTrigger) ||    /* every 50ms */
       gpioSetAlertFunc(SONAR_ECHO, sonarEcho)) {  /* monitor sonar echos */
        fprintf(stderr, "Error al inicializar el sonar!\n");
        return -1;
       }
   return 0;
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
    }
    else duerme_pitando(pd);
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
        cancel_audio = 1;
        return;
    }
    if (pthread_create(&pth, NULL, play_wav, file)) return;
    if (modo == 0) pthread_detach(pth);
    else pthread_join(pth, NULL);    
}




/****************** Funciones del Wiimote **************************/
void wiiErr(cwiid_wiimote_t *wiimote, const char *s, va_list ap)
{
  if (wiimote) printf("Wiimote %d:", cwiid_get_id(wiimote)); else printf("-1:");
  vprintf(s, ap);
  printf("\n");
}


/*** wiimote event loop ***/
static void wiiCallback(cwiid_wiimote_t *wiimote, int mesg_count, union cwiid_mesg mesg[], struct timespec *t)
{
    int i, v_izdo, v_dcho;
    static uint16_t previous_buttons;
    unsigned int bateria;
    static int LEDs[4] = { CWIID_LED1_ON,  CWIID_LED1_ON | CWIID_LED2_ON,
                CWIID_LED1_ON | CWIID_LED2_ON | CWIID_LED3_ON,
                CWIID_LED1_ON | CWIID_LED2_ON | CWIID_LED3_ON | CWIID_LED4_ON };        

    for (i = 0; i < mesg_count; i++) {
        switch (mesg[i].type) {
        case CWIID_MESG_BTN:  // Change in buttons
            mando.buttons = mesg[i].btn_mesg.buttons;
            
            /* ajusta la velocidad del coche y la marca en leds del mando */
            if (previous_buttons&CWIID_BTN_PLUS && ~mando.buttons&CWIID_BTN_PLUS) {
                velocidad += 10;
                if (velocidad > 100) velocidad = 100;
                cwiid_set_led(wiimote, LEDs[velocidad/26]);
            }
            if (previous_buttons&CWIID_BTN_MINUS && ~mando.buttons&CWIID_BTN_MINUS) {
                velocidad -= 10;
                if (velocidad < 0) velocidad = 0;
                cwiid_set_led(wiimote, LEDs[velocidad/26]);
            }
            
            /*** Botones A y B, leen la variable global "velocidad" ***/
            if (mando.buttons&(CWIID_BTN_A | CWIID_BTN_B)) { // if A or B or both pressed
                if (mando.buttons&CWIID_BTN_A) v_izdo=v_dcho=velocidad;
                else v_izdo=v_dcho=-velocidad;  // si vamos marcha atrás, invierte velocidad

                /*** Botones LEFT y RIGHT, giran el coche ***/
                if (mando.buttons&CWIID_BTN_RIGHT) {
                    v_dcho = 0;
                    v_izdo = velocidad+30;
                    if (~mando.buttons&CWIID_BTN_A) v_izdo = - v_izdo;  // si vamos marcha atrás, invierte velocidad
                } 
            
                if (mando.buttons&CWIID_BTN_LEFT) {
                    v_izdo = 0;
                    v_dcho = velocidad+30;
                    if (~mando.buttons&CWIID_BTN_A) v_dcho = - v_dcho;  // si vamos marcha atrás, invierte velocidad
                }
            }
            else v_izdo = v_dcho = 0;
                    
            /*** Ahora, activa la velocidad calculada en cada motor ***/
            ajustaVelocidad(&m_izdo, v_izdo);
            ajustaVelocidad(&m_dcho, v_dcho);
        
    
            /*** pito ***/
            if (~previous_buttons&CWIID_BTN_DOWN && mando.buttons&CWIID_BTN_DOWN) activaPito();    
            if (previous_buttons&CWIID_BTN_DOWN && ~mando.buttons&CWIID_BTN_DOWN) desactivaPito();    
            
            
            /*** sonido ***/
            if (~previous_buttons&CWIID_BTN_UP && mando.buttons&CWIID_BTN_UP) {
                audioplay("sounds/police.wav", 0);
            }
        
            /*** End of buttons loop ***/
            previous_buttons = mando.buttons;
            break;
                    
        case CWIID_MESG_STATUS:
            bateria = (100*mesg[i].status_mesg.battery)/CWIID_BATTERY_MAX;
            printf("Bateria del wiimote: %u\%\n", bateria);
            cwiid_set_led(wiimote, LEDs[velocidad/26]);
            break;
            
        default:
            printf("Mensaje desconocido del wiimote!!\n");
            break;
        }
    }
}


void setupWiimote(void)
{
    cwiid_wiimote_t *wiimote;
    bdaddr_t ba;
    
    if (mando.wiimote) cwiid_close(mando.wiimote);
    cwiid_set_err(wiiErr);
    mando.buttons = 0;
    pito(5, 1);   // Pita 5 décimas para avisar que comienza búsqueda de mando
    gpioSleep(PI_TIME_RELATIVE, 2, 0);  // para desconectar el mando si estaba conectado
    printf("Pulsa las teclas 1 y 2 en el mando de la Wii...\n");
    ba = *BDADDR_ANY;
    wiimote = cwiid_open_timeout(&ba, 0, 5);
    if (!wiimote ||
        cwiid_set_rpt_mode(wiimote, CWIID_RPT_BTN | CWIID_RPT_STATUS) || 
        cwiid_set_mesg_callback(wiimote, wiiCallback) ||
        cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC)) {
            fprintf(stderr, "No puedo conectarme al mando de la Wii!\n");
            mando.wiimote = NULL;
            return;  // No es error si no hay wiimote, el coche funciona sin mando
    } 
    mando.wiimote = wiimote;
    printf("Conectado al mando de la Wii\n");
    pito(2, 1);  // Pita 2 décimas para avisar de que ha concluido con éxito la búsqueda de mando
    cwiid_set_rumble(wiimote, 1);  // señala mediante zumbido el mando sincronizado
    gpioSleep(PI_TIME_RELATIVE, 0, 500000);   // Espera 0,5 segundos para separar zumbido y resto operaciones
    cwiid_set_rumble(wiimote, 0);
    return;
}


/* callback llamado cuando el pin WMSCAN_PIN cambia de estado. Tiene un pull-up a VCC, OFF==pulsado */
void wmScan(int gpio, int level, uint32_t tick)
{
static int button;
    
    switch (level) {
        case PI_ON:   // Sync button released
            if (button != 1) return;   // elimina clicks espureos
            button = 0;
            ajustaVelocidad(&m_izdo, 0);  // Para el coche mientras escanea wiimotes
            ajustaVelocidad(&m_dcho, 0);
            velocidad = 50;  // Nueva velocidad inicial, con o sin mando
            setupWiimote();    
            if (!mando.wiimote) {  // No hay mando, coche es autónomo
                ajustaVelocidad(&m_izdo, velocidad);
                ajustaVelocidad(&m_dcho, velocidad);                
            }
            break;
        case PI_OFF:  // Sync button pressed
            button = 1;
            break;
    }
}




/****************** Funciones auxiliares varias **************************/


/* Rota el coche a la derecha (dextrógiro, sentido>0) o a la izquierda (levógiro, sentido<0)*/
void rota(Motor_t *izdo, Motor_t *dcho, int sentido)  
{
    if (sentido>0) {  // sentido horario
       ajustaVelocidad(izdo, 100);
       ajustaVelocidad(dcho, -100);
    }
    else{
       ajustaVelocidad(izdo, -100);
       ajustaVelocidad(dcho, 100);
    }    
}





/* Comprueba si los motores tienen alimentación.
Devuelve valor de las baterías de alimentación del motor en la variable global powerState: 
PI_OFF si no hay alimentación o es baja, PI_ON si todo OK
Toma 3 muestras espaciadas 400 ms, en bucle, hasta que coincidan.
Esto es necesario porque al arrancar los motores hay un tiempo de oscilación de las baterías, 
aunque está amortiguado por el condensador del circuito */    
void getPowerState(void)
{
    int power1, power2, power3;
    int n = 3;
    
    do {
        power1 = gpioRead(VBAT_PIN);
        if (power1 < 0) return;  // Error al leer, valor inválido
        gpioSleep(PI_TIME_RELATIVE, 0, 400000);  
        power2 = gpioRead(VBAT_PIN);
        if (power2 < 0) return;
        gpioSleep(PI_TIME_RELATIVE, 0, 400000);  
        power3 = gpioRead(VBAT_PIN);
        if (power3 < 0) return;
    } while (power1!=power2 || power2!=power3);
    powerState = power1;  // set global variable
    
    // señal acústica en caso de batería baja
    if (powerState == PI_OFF) {
        while(n--) {
            pito(2, 1);  // pita 2 décimas en este hilo (vuelve después de pitar)
            gpioSleep(PI_TIME_RELATIVE, 0, 200000);  // espera 2 décimas de segundo
        }
    }
}



/* Al recibir una señal, para el coche, cierra todo y termina el programa */
void terminate(int signum)
{
   ajustaVelocidad(&m_izdo, 0);
   ajustaVelocidad(&m_dcho, 0);
   stopMotor(&m_izdo);
   stopMotor(&m_dcho);
   if (mando.wiimote) cwiid_close(mando.wiimote);
   gpioSetPullUpDown(WMSCAN_PIN, PI_PUD_OFF);
   gpioSetPullUpDown(VBAT_PIN, PI_PUD_OFF);  
   gpioWrite(bocina.pin, PI_OFF);
   gpioSetMode(AUDR_PIN, PI_INPUT);
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
    
   gpioSetMode(bocina.pin, PI_OUTPUT);
   gpioWrite(bocina.pin, PI_OFF);
   gpioSetMode(AUDR_PIN, AUDR_ALT);  // Saca PWM0 (audio right) por el GPIO al amplificador
   
   gpioSetMode(VBAT_PIN, PI_INPUT);
   gpioSetPullUpDown(VBAT_PIN, PI_PUD_DOWN);  // pull-down resistor; avoids floating pin if circuit not connected  
   gpioSetTimerFunc(1, 15000, getPowerState);  // Comprueba tensión motores cada 15 seg, timer#1
   getPowerState();    
   
   gpioSetMode(WMSCAN_PIN, PI_INPUT);
   gpioSetPullUpDown(WMSCAN_PIN, PI_PUD_UP);  // pull-up resistor; button pressed == OFF
   gpioGlitchFilter(WMSCAN_PIN, 100000);      // 0,1 sec filter
   
   r |= setupMotor(&m_izdo);
   r |= setupMotor(&m_dcho);
   
   if (powerState == PI_OFF) {
       fprintf(stderr, "La bateria de los motores esta descargada. Coche no arranca!\n");
       ///terminate(SIGINT);
   }    

   setupWiimote();
   gpioSetAlertFunc(WMSCAN_PIN, wmScan);  // Call wmScan when button changes. Debe llamarse después de setupWiimote

   r |= setupSonar();
   r |= sem_init(&semaphore, 0, 0);
   return r;
}


/****************** Main **************************/
void main(int argc, char *argv[])
{
   int r;
   
   r = setup();
   if (r) {
       fprintf(stderr, "Error al inicializar. Coche no arranca!\n");
       if (r < 0) terminate(SIGINT);   // Si el error estaba al inicializar pigpio (r>0), no llames a terminate
       else exit(1);
   }
    
   audioplay("sounds/ready.wav", 1);
   if (mando.wiimote==NULL) {
        ajustaVelocidad(&m_izdo, velocidad);
        ajustaVelocidad(&m_dcho, velocidad);       
   }
   
   for (;;) { 
          esquivando = 0;  // señala que ya se puede volver a enviar la señal de obstáculo encontrado
       r = sem_wait(&semaphore);   // bloquea hasta que encontremos un obstáculo
       if (r) {
            perror("Error al esperar al semaforo");
       }
       
       if (mando.wiimote && ~mando.buttons&CWIID_BTN_A) continue;
       while (distancia < DISTMIN) {
            //printf("Distancia: %d cm\n", distancia); 
            stopMotor(&m_izdo);
            stopMotor(&m_dcho);
            pito(2, 0);  // pita 2 décimas en otro hilo (vuelve inmediatamente)
            gpioSleep(PI_TIME_RELATIVE, 1, 0);
       
            // No esquiva si damos marcha atrás o si ya no pulsamos A
            if (mando.wiimote && ~mando.buttons&CWIID_BTN_A) break;

            // Gira un poco el coche para esquivar el obstáculo
            if (mando.wiimote && mando.buttons&CWIID_BTN_LEFT) rota(&m_izdo, &m_dcho, -1);  // con LEFT pulsado, esquiva a la izquierda
            else rota(&m_izdo, &m_dcho, 1);  // en caso contrario a la derecha
            gpioSleep(PI_TIME_RELATIVE, 0, 500000);
            stopMotor(&m_izdo);
            stopMotor(&m_dcho);
       }
       // Hemos esquivado el obstáculo, ahora velocidad normal si A está pulsada
       if (!mando.wiimote || mando.buttons&CWIID_BTN_A) {
            ajustaVelocidad(&m_izdo, velocidad);
            ajustaVelocidad(&m_dcho, velocidad);
       }
   }
   
   
}




