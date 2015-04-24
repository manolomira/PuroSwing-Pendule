/*
SKETCH PARA MODULO PENDULE DE LA EXPOSICION PURO SWING
Adaptado para su uso con Shield Pendulo (version inicial)

Version 26 de septiembre 2014:
.- Añadido el control de amplitud.
.- Corregido el calculo de desfase de la rueda. Se regula con el potenciometro central, unido a A1
.- Extra Ton sigue ajustandose con A0

Disponible en GitHub: https://github.com/manolomira/PuroSwing-Pendule

Basado en sistema de pendulo perpetuo
28-Noviembre-2013

Sensores de barrera optica para el pendulo a trav´s de placa 
    amplificadora
Sensor Hall para la rueda

Salida PWM para controlar un modulo basado en controlador en puente L298
*/


#include <Bounce.h>

// *************** VARIABLES DEL PENDULO ***************

int amplitud, amplitud_max= 38;





// Define entradas y salidas
const int SWInner = 2;       // Entrada de la barrera optica de referencia 
                             // del pendulo
const int LEDInner = 3;      // Indica que se activo la barrera optica de 
                             // referencia del pendulo

const int FinIman = 4;       // Entrada de la barrera optica que indica que 
                             // el pendulo esta cerca del electroiman

const int ElectMag = 5;      // Salida que activa el electriman

// la SALIDA   6 esta reservada para indicar que se activo la barrera FinIman
// La ENTRADA  7 esta reservada como libre1 en el conector SENS_PEND

const int LED_Enclavado = 9; // Salida que indica modo enclavado (L1)

const int LEDElectMag = 11;  // ****** PROVISIONAL *********
                             // Indica que el electroiman esta activado

// la ENTRADA 12 esta reservada como libre2 en el conector SENS_PEND

const int pot1 = A0;         // Entrada analogica potenciometro de ajuste P1
const int pot2 = A1;         // Entrada analogica potenciometro de ajuste P2
const int pot3 = A2;         // Entrada analogica potenciometro de ajuste P3
const int eAnalog = A3;      // Entrada analogica de ajuste

// Las entradas analogicas A4 y A5 se reservan para su uso como bus I2C



const long intervaloSeguridad = 1000;        // tiempo maximo que se activa el electroiman por seguridad


Bounce SWInnerBounce = Bounce( SWInner,50 ); 
Bounce FinImanBounce = Bounce( FinIman,50 ); 


long milliseconds;
long tiempoActual;
long tiempoAnterior;
long tiempoOn;
long tiempoOff;
  
int periodoPendulo;
int intervaloActual;
int intervaloAnterior;
int extraTon;                          // tiempo extra calculado a partir del potenciometro


// *************** VARIABLES DE LA RUEDA ***************


const int minimoPWM  = 100;           // Valor minimo de salida del PWM al motor
const int maximoPWM  = 255;

// Define entradas y salidas
const int int_pendulo =  3;    // Entrada del pulso del pendulo (periodo fijo)
const int int_rueda   =  8;    // Entrada del pulso de la rueda (periodo a regular)

const int out_motor1  = 10;    // Salida PWM de control de la velocidad de la rueda

// const int out_motor2  =  6;    // Salida PWM para control inverso

//  Inicializacion de variables

Bounce SWRuedaBounce = Bounce( int_rueda,250 ); 


boolean periodoRuedaEnclavado = false; // indica que el periodo esta enclavado
int contadorRueda = 10;                // vueltas de la rueda antes de iniciar el ajuste de velocidad

float gananciaNoEnclavado = 0.13;
float gananciaEnclavado = 0.15;


int diferenciaPeriodoRueda;
int salidaMotorEnclavado;
int inicioContadorEnclavado;
int  periodoRueda;
long tiempoRuedaAnt;
long tiempoRuedaAct;

int salidaPWMRueda;

int deltasMotor[]  = {20, 20, 20, 20, 20};
int salidasMotor[] = {20, 20, 20, 20, 20};

int extraDesfase;


void setup()
{
// *************** VARIABLES DEL PENDULO ***************
  pinMode(SWInner, INPUT);        // optoacoplador de llegada al REED
  pinMode(FinIman,INPUT);         // optoacoplador de llegada al electroiman

  pinMode(LEDInner, OUTPUT);      // indica la activacion del del REED
  pinMode(LEDElectMag, OUTPUT);   // indica la activacion del electroiman
  pinMode(ElectMag, OUTPUT);      // Salida electroiman
  pinMode(LED_Enclavado, OUTPUT); // Salida que indica modo enclavado (L1)
  

  // *************** VARIABLES DE LA RUEDA ***************
  pinMode(int_pendulo, INPUT);
  pinMode(int_rueda,   INPUT);    // Sensor del giro de la rueda

  pinMode(out_motor1, OUTPUT);    // Salida al motor por PWm
 // pinMode(out_motor2, OUTPUT);
  
  salidaPWMRueda = minimoPWM;

  milliseconds = millis();

  // Señaliza el estado enclavado por medio del LED L1
  if (periodoRuedaEnclavado == true)  {digitalWrite (LED_Enclavado, HIGH);}
  else  {digitalWrite (LED_Enclavado, LOW);}
  
  Serial.begin(9600);   
}


//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
void loop()
{
   
  // *************** GESTION DEL PENDULO ***************

  
  SWInnerBounce.update ();       // Actualiza las instalacias Bounce del pendulo y detecta flancos de bajada.
  FinImanBounce.update (); 

  ActualizaPendulo();             // Actualiza las variables asociadas al pendulo y enciende sus actuadores

  



  // *************** GESTION DE LA RUEDA ***************

  SWRuedaBounce.update ();                               // Actualiza las instalacias Bounce del detector del giro de la rueda.
  boolean pulsoRueda = SWRuedaBounce.fallingEdge();      // Detecta flancos de bajada en el detector.
  
  if (pulsoRueda == true)                                // Si se produce un peso por el sensor ctualiza variables
  {
    contadorRueda ++;

    tiempoRuedaAnt = tiempoRuedaAct;
    tiempoRuedaAct = millis ();
    
    periodoRueda = tiempoRuedaAct - tiempoRuedaAnt;              // Calcula el periodo de la rueda
    
    
    // Ajusta la salida del motor
    
                                                                 // Ajuste cuando el periodo de la rueda y el del pendulo son distintos.
                                                                 // La salida PWM se incrementa con un valor proporcional a la 
                                                                 //       diferencia de periodos
                                                                 // Al salir de este modo se establece el valor de salidaMotorEnclavado
    if (periodoRuedaEnclavado == false) 
    salidaPWMRueda = calculo_salida_no_enclavado(salidaPWMRueda);

    
                                                                  // Ajuste cunado el periodo de la rueda y el del pendulo SON iguales
                                                                  // La salida PWM se calcula como salidaMotorEnclavado + un valor proporcional
                                                                  //        a la diferencia entre el momento en el que se producen los 
                                                                  //        pulsos de control
    else
    salidaPWMRueda = calculo_salida_enclavado();
  }
  
  if (salidaPWMRueda < minimoPWM) salidaPWMRueda = minimoPWM;
  if (salidaPWMRueda > maximoPWM) salidaPWMRueda = maximoPWM;

  analogWrite(out_motor1, salidaPWMRueda);  
}
  
    
  
 
//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
int calculo_salida_no_enclavado(int salidaMotorRueda)
{
  int diferenciaPeriodoRueda;
  float deltaSalidaMotor;
  float deltaMotorMedia = 0.0;
  float deltaMotorMaxim = 0.0;
  float salidaMotorMedia = 0.0;

  
  digitalWrite(LED_Enclavado, LOW);
  
  diferenciaPeriodoRueda = periodoRueda - periodoPendulo;
  deltaSalidaMotor = diferenciaPeriodoRueda * gananciaNoEnclavado;
        if (deltaSalidaMotor >  15.0) deltaSalidaMotor =  15.0;
        if (deltaSalidaMotor < -15.0) deltaSalidaMotor = -15.0;

      salidaMotorRueda += deltaSalidaMotor;
 
      
      deltaMotorMedia  = 0;                                       // Calcula la media de los 5 ultimos valores de deltaSalidaMotor
      deltaMotorMaxim  = 0;                                       // el valor maximo de los 5 ultimos valores de deltaSalidaMotor
      salidaMotorMedia = 0;                                       // el valor medio de los 5 ultimos valores de salidaMotorRueda
      for (int i = 0; i < 4; i++)
      {
        deltasMotor[i]  =  deltasMotor [i+1];
        deltaMotorMedia += deltasMotor[i];
        
        deltaMotorMaxim = max(deltaMotorMaxim , abs(deltasMotor[i]));
        
        salidasMotor[i]  =  salidasMotor [i+1];
        salidaMotorMedia += salidasMotor[i];
        
      }
      deltasMotor[4]  = deltaSalidaMotor;
      deltaMotorMedia += deltasMotor[4];
      deltaMotorMedia /= 5.0;
 
      deltaMotorMaxim = max(deltaMotorMaxim , abs(deltasMotor[4]));
      
      salidasMotor[4]  =  salidaMotorRueda;
      salidaMotorMedia += salidasMotor[4];
      salidaMotorMedia /= 5.0;
      
                                                                   // Se inicia el modo enclavado si la media de las desviaciones 
                                                                   //         es menor que 5 y ninguna es de mas de 10
      if (abs(deltaMotorMedia < 5.0) && (deltaMotorMaxim < 10))
      {
        salidaMotorEnclavado = salidaMotorMedia;
        periodoRuedaEnclavado = true;
        inicioContadorEnclavado = contadorRueda;
      }
  
      Serial.print ("                    NO ENCLAVADO. PeriodoRueda = ");
      Serial.print (periodoRueda);
      Serial.print (" diferencia = ");
      Serial.print (diferenciaPeriodoRueda);
      Serial.print (" >> salidaPWM = ");
      Serial.print (salidaMotorRueda);
      Serial.print (" (+");
      Serial.print (deltaSalidaMotor);
      Serial.print (")");
      Serial.println ();
  
      return salidaMotorRueda;
}


//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
int calculo_salida_enclavado()
{
  int diferenciaRueda;
  int salidaMotorRueda;
  float deltaSalidaMotor;
  
  digitalWrite(LED_Enclavado, HIGH);
  
  extraDesfase = map(analogRead(A1), 0, 1023, -500, 500);
  // int extraDesfase = map(analogRead(A0), 0, 1023, -500, 500);

  
  int diferenciaOriginal;
  
     diferenciaRueda = tiempoRuedaAct - tiempoActual - 500 - extraDesfase;    
                                                                  // Calcula el desfase entre la rueda y el pendulo. 
                                                                  // Busca el punto en el que tengan un desfase de 500 ms
     diferenciaOriginal = diferenciaRueda;
        if (diferenciaRueda > periodoRueda / 2)                   //     Solo es significativo si los periodos son iguales
        { diferenciaRueda -= periodoRueda;}

      deltaSalidaMotor = diferenciaRueda * gananciaEnclavado;
        if (deltaSalidaMotor >  5.0) deltaSalidaMotor =  4.0;
        if (deltaSalidaMotor < -5.0) deltaSalidaMotor = -4.0;
        

      if (contadorRueda > inicioContadorEnclavado + 50)            // Sale del modo enclavado tras 20 pasos de ajuste
      {
        salidaMotorRueda = salidaMotorEnclavado + deltaSalidaMotor;
        periodoRuedaEnclavado = false;
        deltasMotor[4] = 20;                                      // Fuerza a que se produzcan 5 pasos no enclavados
        
      }
      else
      {
        salidaMotorRueda = salidaMotorEnclavado + deltaSalidaMotor;
      }
 
  
      Serial.print ("                    **ENCLAVADO** PeriodoRueda = ");
      Serial.print (periodoRueda);
 /*     
      Serial.print (" Trueda = ");
      Serial.print (tiempoRuedaAct);
     Serial.print (" tpendulo = ");
      Serial.print (tiempoActual);
      
      
      Serial.print (" delta T0 = ");
      Serial.print (diferenciaOriginal);
   
   */
      Serial.print (" delta T = ");
      Serial.print (diferenciaRueda);
      Serial.print (" + ");
      Serial.print (extraDesfase);
      Serial.print (" >> salidaPWM = ");
      Serial.print (salidaMotorEnclavado);
      Serial.print (" (+");
      Serial.print (deltaSalidaMotor);
      Serial.print (")");
      Serial.println (); 
  
      return salidaMotorRueda;
}


//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
void ActualizaPendulo()
{

  int tiempoElectroImnan;
  boolean pulsoControl = SWInnerBounce.fallingEdge();      // Detecta flancis de bajada en los detectores opticos.
  boolean pulsoFinIman = FinImanBounce.fallingEdge();
  boolean pulso_si = false;
  
  milliseconds = millis();                                 // Actualiza el contador de tiempo.
                                                           // Actualiza el ajuste de tiempo extra de encendido de electroiman.
  extraTon = map(analogRead(A0), 0, 1023, -10, 50);  if (extraTon < 0) extraTon = 0;


  if (pulsoControl == true)                                // Cuando detecta el pulso de control actualiza los tiempos.
  {
    tiempoAnterior = tiempoActual;
    tiempoActual   = milliseconds;
    
    intervaloAnterior = intervaloActual;
    intervaloActual = tiempoActual - tiempoAnterior;

    periodoPendulo = intervaloAnterior + intervaloActual;

    digitalWrite(LEDInner, HIGH);                          // Enciende LEDinner como referencia cuando se activa el sensor 

                                                           // intervaloActual > intervaloAnterior es la condicion 
                                                           //      cuando el pendulo esta de vuelta por lo que activa
                                                           //      el electroiman.   
    if ((intervaloAnterior > 0) && (intervaloActual < intervaloAnterior))   
    {
      if (amplitud < amplitud_max)
      {
        digitalWrite (ElectMag, HIGH);                       // Conecta el electroiman y LEDElectroMag como control
        digitalWrite (LEDElectMag, HIGH);
      }
      pulso_si = true;
      tiempoOn = milliseconds;                             // Registra el momento de conexion.   
      tiempoOff = milliseconds + intervaloSeguridad;       // Inicialmente fija tiempoOff al limite del intervalo de seguridad


 /*     Serial.print ("    tAnterior = ");
      Serial.print (tiempoAnterior);
      Serial.print ("    tActual = ");
      Serial.print (tiempoActual);
            Serial.print ("    tpo = ");
      Serial.print (intervaloAnterior);
      Serial.print (" + ");
      Serial.print (intervaloActual);
    
      Serial.println ();
   */   

    } 
    

  }
 
                                                            // Apaga LEDInner 15 ms despues de activarlo.
  if (milliseconds > (tiempoActual + 15))   digitalWrite(LEDInner, LOW);        


  if ((pulsoFinIman == true))                               // Registra el paso por el detector de apagado del Electroiman. 
  {
    tiempoOff = milliseconds;                               // Establece el valor real para tiempoOff. 
  }

                                                            // Apaga el Electroiman pasado pasado el tiempo de ajuste fino 
                                                                     // tras la activacion del detector de fin o cuando 
                                                                     // haya pasado el intervalo de Seguridad
    tiempoElectroImnan = tiempoOff - tiempoOn;
    extraTon = tiempoElectroImnan / 6 ;
     
  if ((digitalRead (ElectMag) == HIGH) && ((milliseconds > (tiempoOn + intervaloSeguridad)) || (milliseconds > (tiempoOff + extraTon))))
  {
      digitalWrite (ElectMag, LOW);                         // Desconecta el electroiman
      digitalWrite (LEDElectMag, LOW);
           
                                                            // Despues imprime los datos del ciclo completado. 
  }
      
  if (pulso_si == true)
  {  
      amplitud = 0.0002 * intervaloActual * intervaloActual - 0.1472 * intervaloActual + 49.125;
 
      Serial.print ("    tpo = ");
      Serial.print (intervaloAnterior);
      Serial.print (" + ");
      Serial.print (intervaloActual);
      Serial.print (" >> periodo = ");
      Serial.print (periodoPendulo);
      Serial.print ("   ttot = ");
      Serial.print (tiempoOff - tiempoOn);
      Serial.print (" + ");
      Serial.print (extraTon);
      Serial.print ("  >> amplitud = ");
      Serial.print (amplitud);
      Serial.print ("  amp_max = ");
      Serial.print (amplitud_max);
      Serial.print ("    extra_desfase = ");
      Serial.print (extraDesfase);
      Serial.println();
      Serial.println();

      pulso_si = false;      
  }
}
