/*
SKETCH PARA MODULO PENDULE DE LA EXPOSICION PURO SWING

Disponible en GitHub: https://github.com/manolomira/PuroSwing-Pendule

Revision 12 de enero de 2019:
.- Ajustado el valor de minimoPWM para adaptarse al nuevo motor
.- Ahora el motor es Ref 915-8915  https://es.rs-online.com/web/p/motores-dc-con-caja-reductora/9158915/
            es de MAXON con referencia 123854
            
Version 26 de septiembre 2014:
.- Añadido el control de amplitud.
.- Corregido el calculo de desfase de la rueda. Se regula con el potenciometro central, unido a A1
.- Extra Ton sigue ajustandose con A0

Adaptado para su uso con Shield Pendulo (version inicial)
Basado en sistema de pendulo perpetuo
28-Noviembre-2013

Sensores de barrera optica para el pendulo a través de placa amplificadora
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

// La SALIDA   6 esta reservada para indicar que se activo la barrera FinIman
// La ENTRADA  7 esta reservada como libre1 en el conector SENS_PEND

const int LED_Enclavado = 9; // Salida que indica modo enclavado (L1)

const int LEDElectMag = 11;  // ****** PROVISIONAL *********
                             // Indica que el electroiman esta activado

// la ENTRADA 12 esta reservada como libre2 en el conector SENS_PEND

const int pot1 = A0;         // potenciometro ajuste Extra TON (ANULADO)
const int pot2 = A1;         // potenciometro ajuste Extra desfase
const int pot3 = A2;         // P3 (sin uso)
const int eAnalog = A3;      // Entrada analogica de ajuste
// Las entradas analogicas A4 y A5 se reservan para su uso como bus I2C

const long intervaloSeguridad = 1000;        // tiempo maximo que se activa el electroiman por seguridad

Bounce SWInnerBounce = Bounce( SWInner,50 ); // Referencia del pendulo
Bounce FinImanBounce = Bounce( FinIman,50 ); // Cerca del electroiman

long milliseconds;
long tiempoActual;          // OBSOLETA
long tiempoAnterior;          // OBSOLETA
long tiempoOn;          // OBSOLETA
long tiempoOff;          // OBSOLETA
  
int periodoPendulo;
int intervaloActual;     // OBSOLETA
int intervaloAnterior;    // OBSOLETA
int extraTon;              // OBSOLETA            // tiempo extra calculado a partir del potenciometro

unsigned long T_Pendulo;

// *************** VARIABLES DE LA RUEDA ***************


const int minimoPWM  = 100;           // Valor minimo de salida del PWM al motor
const int maximoPWM  = 255;

// Define entradas y salidas
const int int_rueda   =  8;    // Entrada del pulso de la rueda (periodo a regular)

const int out_motor1  = 10;    // Salida PWM de control de la velocidad de la rueda

//  Inicializacion de INSTANCIAS

Bounce SWRuedaBounce = Bounce( int_rueda,250 ); 

boolean periodoRuedaEnclavado = false; // indica que el periodo esta enclavado
int contadorRueda = 10;                // vueltas de la rueda antes de iniciar el ajuste de velocidad

float gananciaNoEnclavado = 0.13;
float gananciaEnclavado   = 0.05;


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
  pinMode(int_rueda,   INPUT);    // Sensor del giro de la rueda

  pinMode(out_motor1, OUTPUT);    // Salida al motor por PWm
   
  salidaPWMRueda = (minimoPWM + maximoPWM) / 2;

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

  ActualizaPendulo();            // Actualiza las variables asociadas al pendulo y enciende sus actuadores

  



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
int calculo_salida_no_enclavado (int salidaMotorRueda)
{
  // Calcula el nuevo valor de la salida PWM que controla el motor
       // cuando aun la rueda aun no alcanzo la velocidad del pendulo
  // salidaMotorRueda es el valor actual de la salida PWM
  // La salida es el nuevo valor para la salida PWM
  
  
  int diferenciaPeriodoRueda;
  float deltaSalidaMotor;
  float deltaMotorMedia = 0.0;
  float deltaMotorMaxim = 0.0;
  float salidaMotorMedia = 0.0;

  digitalWrite(LED_Enclavado, LOW);
  
  diferenciaPeriodoRueda = periodoRueda - periodoPendulo;
  deltaSalidaMotor = diferenciaPeriodoRueda * gananciaNoEnclavado;
      if (deltaSalidaMotor >  15.0) deltaSalidaMotor =  10.0;
      if (deltaSalidaMotor < -15.0) deltaSalidaMotor = -10.0;

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
  
      Serial.print ("\t          NO ENCLAVADO. PeriodoRueda = ");
      Serial.print (periodoRueda);
      Serial.print ("\tdiferencia =   ");
      Serial.print (diferenciaPeriodoRueda);
      Serial.print ("\t>> salidaPWM = ");
      Serial.print (salidaMotorRueda);
      Serial.print (" (+");
      Serial.print (deltaSalidaMotor);
      Serial.print (")");
      Serial.println ();
      Serial.println ();
  
      return salidaMotorRueda;
}


//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
int calculo_salida_enclavado()
{
  // Calcula el nuevo valor de la salida PWM que controla el motor
       // cuando aun la rueda alcanzo la velocidad del pendulo
  // salidaMotorRueda es el valor actual de la salida PWM
  // La salida es el nuevo valor para la salida PWM
  

  int diferenciaRueda;
  int salidaMotorRueda;
  float deltaSalidaMotor;
  
  digitalWrite(LED_Enclavado, HIGH);
  
  extraDesfase = map(analogRead(pot2), 0, 1023, -500, 500);
  
     diferenciaRueda = tiempoRuedaAct - T_Pendulo - 300;    
                       // Calcula el desfase entre la rueda y el pendulo. 
                       // asume un periodo de 2000 ms, y que el detector de la rueda se activa aproximadamente 
                       // al paso del pendulo por su posicion central

     if (diferenciaRueda > periodoRueda)     { diferenciaRueda -= periodoRueda;}
     if (diferenciaRueda > periodoRueda/2)     { diferenciaRueda -= periodoRueda;}

      deltaSalidaMotor = (diferenciaRueda + extraDesfase) * gananciaEnclavado;
      deltaSalidaMotor = constrain (deltaSalidaMotor, -15, 15);


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
 
  
      Serial.print ("**ENCLAVADO**  PerRueda = ");   Serial.print (periodoRueda);

      Serial.print ("\tTRueda =   ");                Serial.print (tiempoRuedaAct);

      Serial.print ("\tdif. = ");                    Serial.print (diferenciaRueda);
      if (extraDesfase >= 0) Serial.print (" +");    
      else Serial.print (" ");                       Serial.print (extraDesfase);

      Serial.print ("\t >> salidaPWM = ");           Serial.print (salidaMotorEnclavado);
      
      if (deltaSalidaMotor >= 0) Serial.print (" (+");
      else Serial.print (" (");                      Serial.print (deltaSalidaMotor);

      Serial.print (")");
      Serial.println (); 
      Serial.println (); 
  
      return salidaMotorRueda;
}


//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
void ActualizaPendulo()
{
  
  static int zona;
         // zona indica en que zona del recorrido se encuentra el pendulo
         // zona = 0 es la zona opuesta al sensor de control
         // zona = 1 es desde el paso por el centro hasta el sensor de control
         //          calcula los parametros e imprime resultados en esta
         //          zona para evitar retrasar el apagado del pendulo
         // zona = 2 es la zona mas alla del sensor de control
         // zona = 3 es la zona entre el sensor de control y el centro
         //          EN ESTA ZONA SE PRODUCE EL ACTIVAMIENTO DEL ELECTROIMAN
  
  static unsigned long T_0, T_1, T_2, T_3;                   // marca de tiempo de entrada en cada zona
  static unsigned long T_ON, T_Seguridad, T_OFF;             // marca de tiempo para los eventos eventos del pendulo
  int extraT_ON;
 
  static int T_zona_0, T_zona_1, T_zona_2, T_zona_3;         // tiempo en cada zona (milisegundos)
  
  unsigned long milisegundos;                                // Recoge el valor de millis() en cada llamada a la funcion
  milisegundos = millis();
  
  boolean pulsoControl = SWInnerBounce.fallingEdge();      // Detecta flancos de bajada en los detectores opticos.
  boolean pulsoCentro  = FinImanBounce.fallingEdge();


  // *** APAGA EL ELECTROIMAN Y EL PENDULO ASOCIADO ***
  if (milisegundos > min (T_Seguridad, T_OFF)) digitalWrite (ElectMag, LOW);                         // Desconecta el electroiman
  if (milisegundos > (T_ON + 15))              digitalWrite (LEDElectMag, LOW);
 
  
  // *** ejecuta las transiciones entre los zonas en funcion de los detectores opticos
  //     en cada cambio de zona registra la marca de tiempo de salida y el tiempo que permanecio en ella
  //     limita el valor registrado de permanencia en cada zona a 10.000 por conpatibilidad con el tipo int 

  switch (zona) {
    case 0:
      if (pulsoCentro)
        {T_1 = milisegundos;  T_zona_0 = min (10000, T_1 - T_0); zona = 1;
        
         // *** HACE LOS CALCULOS EN ESTA ZONA PARA NO RETRASAR EL APAGADO DEL ELECTRIMAN *** 
         // *** CALCULO DE VARIABLES ASOCIADAS AL PENDULO ***

         amplitud = 0.0002 * T_zona_2 * T_zona_2 - 0.1472 * T_zona_2 + 49.125;
         periodoPendulo = T_zona_0 + T_zona_1 + T_zona_2 + T_zona_3;
         T_Pendulo = T_0;            // corresponde al momento del paso por el centro en direccion 
                                     // contraria a la zona del sensor de control


         // *** IMPRESION DE VALORES ASOCIADOS AL PENDULO ***
         Serial.println("--------------------------------");
         Serial.print ("|  PerPend = ");     Serial.print (T_zona_0 + T_zona_1 + T_zona_3);
         Serial.print (" + ");               Serial.print (T_zona_2);
         Serial.print (" = ");               Serial.print (periodoPendulo);

         Serial.print ("\tTpendulo = ");     Serial.print (T_Pendulo);

         Serial.print ("\tT_iman =  ");        Serial.print (T_zona_3);
         Serial.print (" +");               Serial.print (extraT_ON);

         Serial.print ("\t >> amplitud = "); Serial.print (amplitud);
         Serial.print (" (");                Serial.print (amplitud_max);
         Serial.print (")");

         Serial.println ();
         Serial.println ("--------------------------------");
         
         
         Serial.print   ("tiempo de impresion = ");
         Serial.println (millis () - T_1);
      }
      break;
 
    case 1:
      if (pulsoControl)
        {T_2 = milisegundos;  T_zona_1 = min (10000, T_2 - T_1); zona = 2;}
      break;
 
    case 2:
      if (pulsoControl) 
        {T_3 = milisegundos;  T_zona_2 = min (10000, T_3 - T_2); zona = 3;        

        // *** AL ENTRAR EN ZONA 3 SE PRODUCE EL ACTIVAMIENTO DEL ELECTROIMAN  ***
        T_Seguridad = T_3 + 1000;             // establece el maximo tiempo encendido del electroiman
        T_OFF       = T_Seguridad;            // establece provisionalmente el momento de apagado del electroiman

       if (amplitud < amplitud_max)          // solo enciende electroiman si la amplitud es menor de la establecida
        {
          digitalWrite (ElectMag, HIGH);      // Conecta el electroiman y LEDElectroMag como control
          digitalWrite (LEDElectMag, HIGH);
        }
      }
      break;
      
    case 3:
      if (pulsoCentro)
      { T_0 = milisegundos;  T_zona_3 = min (10000, T_0 - T_3); zona = 0;


       // *** AL SALIR DE LA ZONA 3 SE CALCULA EL MOMENTO DE APAGADO DEL ELECTROIMAN ***
       extraT_ON = T_zona_3 / 6;
       extraT_ON = map (analogRead (pot1), 0, 1023, 100, 0);
       T_OFF     = T_0 + extraT_ON;                            // establece el valor correcto para T_OFF

     }
       break;
    default: 
      zona = 0;  // si por algun motivo la variable se sale de margenes vuelve a la zona 0
  }
}





//  ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** ********** **********  
void ActualizaPendulo_antigua()
{

  int tiempoElectroImnan;
  boolean pulsoControl = SWInnerBounce.fallingEdge();      // Detecta flancos de bajada en los detectores opticos.
  boolean pulsoFinIman = FinImanBounce.fallingEdge();
  boolean pulso_si = false;
  
  milliseconds = millis();                                 // Actualiza el contador de tiempo.
                                                           // Actualiza el ajuste de tiempo extra de encendido de electroiman.
  extraTon = map(analogRead(pot1), 0, 1023, -10, 50);  if (extraTon < 0) extraTon = 0;


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

      /*
      Serial.print ("    tAnterior = ");
      Serial.print (tiempoAnterior);
      Serial.print ("    tActual = ");
      Serial.print (tiempoActual);
            Serial.print ("    tpo = ");
      Serial.print (intervaloAnterior);
      Serial.print (" + ");
      Serial.print (intervaloActual);
    
      Serial.println ();
      Serial.println ();
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
                                                            // Apaga el Electroiman pasado  el tiempo de ajuste fino 
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
 
      Serial.println("--------------------------------");
      Serial.print ("|  PerPend = ");     Serial.print (intervaloAnterior);
      Serial.print (" + ");               Serial.print (intervaloActual);
      Serial.print (" = ");               Serial.print (periodoPendulo);

      Serial.print ("\tTpendulo = ");     Serial.print (tiempoActual);

      Serial.print ("\tTiman = ");        Serial.print (tiempoOff - tiempoOn);
      Serial.print (" + ");               Serial.print (extraTon);

      Serial.print ("\t >> amplitud = "); Serial.print (amplitud);
      Serial.print (" (");                Serial.print (amplitud_max);
      Serial.print (")");
      
      Serial.println();
      Serial.println("--------------------------------");

      pulso_si = false;      
  }
}
