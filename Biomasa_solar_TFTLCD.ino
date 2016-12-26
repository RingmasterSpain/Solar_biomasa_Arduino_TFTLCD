/*
 * ARCHIVO: Control sistema térmico mixto biomasa-solar
 *   AUTOR: David Losada
 *   FECHA: 8/08/2016
 *     URL: http://miqueridopinwino.blogspot.com.es/2016/08/control-centralizado-del-sistema-mixto-biomasa-solar-con-Arduino.html
 *   Versión 1.6 (15/12/16)
 *   - Se ha corregido el código de desconexión en caso de biomasa encendida, para evitar apagar el motor demasiado pronto con brasas
 *   - Se añade aviso de excesiva diferencia entre sondas, indicando en rojo la temperatura y activando alarma
 *   - Mejorada la alarma; se hace intermitente.
 *   - Añadido un 5º termistor en el serpentin biomasa (parte baja) por seguridad; Biomasa2, a veces se calienta antes en la parte inferior
 *   - Corregida la contabilización de horas funcionamiento
 *   - Cambiado el orden en pantalla
 *   - Se indican los KWh ahorrados por energía renovable :) (aproximado)
 *   - Cambiada la forma en que se comprueban temperaturas, dando prioridad real a biomasa (o estamos jodidos)
 *   - Ahora se activa cada 15 días la válvula de enfriado para evitar su agarrotamiento
 *   - Corregidos varios errores
 *   - 10/12/16 Mejorados varios puntos; no contabilizaba horas de motor, y la comprobación de temperaturas no era óptima con depósito
 *   - 12/12/16 Mejorado el código: añadido código para dormir el procesador, reducir los refrescos a lo mínimo necesario y sensor de pellets futuro
 *   - 15/12/16 Corregida la evaluación del tiempo
 *
 * OBJETIVO: Prototipo control de sistema casero mixto biomasa-solar térmica
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

// Uses TFTLCD sketch that has been Refurbished by BUHOSOFT
// If using an Arduino Mega make sure to use its hardware SPI pins, OR make
// sure the SD library is configured for 'soft' SPI in the file Sd2Card.h.

// Original code provided by Smoke And Wires
// http://www.smokeandwires.co.nz
// This code has been taken from the Adafruit TFT Library and modified
//  by us for use with our TFT Shields / Modules
// For original code / licensing please refer to
// https://github.com/adafruit/TFTLCD-Library

// adapted sketch by niq_ro from http://arduinotehniq.blogspot.com/
// ver. 1m5 - 13.11.2014, Craiova - Romania

//Librerías para dormir al procesador y ahorrar energía cuando no hace nada
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

//Librerías para la pantalla gráfica
#include <pin_magic.h>
#include <pin_magic_MEGA.h>
#include <pin_magic_UNO.h>
#include <registers.h>
#include <TFTlcd.h>
#include <TouchScreen.h>

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
// #define LCD_CS A3 // Chip Select goes to Analog 3
// #define LCD_CD A2 // Command/Data goes to Analog 2
// #define LCD_WR A1 // LCD Write goes to Analog 1
// #define LCD_RD A0 // LCD Read goes to Analog 0

// #define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

//#define DEBUG
#include <Adafruit_GFX.h>
#include <EEPROMex.h> //Para usar la memoria EEPROM
//Arduino con Tiny RTC I2C http://zygzax.com
//#include <Wire.h>
//#include <DS1307RTC.h>
#include <Time.h> //Manejo reloj interno Arduino

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4


#define YP A1 // Y+ is on Analog1
#define XM A2 // X- is on Analog2
#define YM 7 // Y- is on Digital7
#define XP 6 // X+ is on Digital6

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 326 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 326);

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
//#define ROZ     0xFD20
#define ROZ     0xFBE0
#define GRI     0xBDF7
// http://stackoverflow.com/questions/13720937/c-defined-16bit-high-color
// http://wiibrew.org/wiki/U16_colors

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);;
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
//Adafruit_TFTLCD tft;



////***************** COMIENZO DE PARÁMETROS CONFIGURABLES ***********************
//Control temperatura
//Para ahorrar cálculos, lo vamos a calcular antes del programa
//const float TPto1 = -6; //Temperatura en ºC punto 1 (A descomentar las siguientes cuatro líneas  si desconocemos el varor de B para el termistor utilizado)
//const float RPto1 = 40000; // Ohms en punto 1
//const float TPto2 = 96; // Temp. punto 2
//const float RPto2 = 970; // Ohms en punto 2
const long B = 3850; //Valor Beta del Datasheet; comentar esta línea si se meten los datos de los dos puntos
const int resistor = 6800; //El valor en ohmnios de la resistencia del termistor a 25ºC
const float voltage = 5.01; // El voltaje real en el punto 5Vcc de tu placa Arduino

//Deltas de temperatura
const byte DtFsolar = 6; //Delta temp ºC conexión de motor circuito panel solar
const byte Dtosolar = 1; //Delta temp. desconexión motor

const byte DtFbiomasa = 12; //Delta temp ºC conexión de motor y válvula circuito biomasa
const byte Dtobiomasa = 2; //Delta temp ºC desconexión de motor y válvula circuito biomasa

const byte DtFEnfriar = 65; //Temp. ºC conexión electroválvula circuito desvío exceso temp a calefacción
const byte DtoEnfriar = 60; //Temp. desconexión electroválvula desvío agua caliente a calefacción

const int frecseg = 1; //Tiempo en segundos X2 entre cada ejecución del programa (recomendado poner entre 1 y 3 para mantener la velocidad de respuesta)
const byte difTemp = 25; //Diferencia de temperatura máxima entre sondas considerada normal, a partir de la cual salta la alarma
const double ltsdeposito = 250; //para el cálculo de Kwh ahorrados; indicar aquí la capacidad del depósito de inercia

//Definición pines digitales para activar relés
#define ledPIN 13 //Led placa arduino
#define motorPIN 25 //Pin digital activar relé motor
#define valvbioPIN 27 //Pin digital activar relé electroválvula circuito biomasa
#define valvenfriaPIN 29 //Pin digital activar relé electroválvula circuito calefacción radiante
#define zumbadorPIN 31 //Conexión al Mosfet del zumbador electrónico de aviso
//En mi caso lo sensores están puestos en este orden desde el 11: Captador,Deposito,Biomasa1,Retorno,Biomasa2
#define primerSensor 11 //Pines analógicos para termistores; van correlativos desde este
#define numeroSensores 5 //Cantidad de sensores

float calibTemps[numeroSensores] = {0,0,0,0,0}; //Matriz de ajustes calibración sensores en ºC; se sumará a la temp. obtenida

int horasValvEnfria=120; //Cada este número de horas/3 se activará una vez la válvula de enfriado para evitar que se quede agarrotada

//***************** FIN DE PARÁMETROS CONFIGURABLES ***********************

//Variables 
double Msensores[numeroSensores] = {0,0,0,0,0}; //Matriz temperaturas obtenidas de sensores en orden de PIN
double Mtempsens[numeroSensores] = {0,0,0,0,0}; //Matriz temperaturas ordenadas a nuestro gusto
double MtempAnt[numeroSensores] = {0,0,0,0,0}; //Matriz temperaturas anteriores para borrado pantalla
const char* Mnombres[]={"Captador:","Biomasa1:","Biomasa2:","Retorno: ","Deposito:"}; //Matriz de arrays (*=pointers) de los nombres de los sensores
String TextoAnt;

#define captador 0 //Definimos la posición en la matriz de cada texto 
#define biomasa1 1
#define biomasa2 2
#define retorno  3
#define deposito 4
#define pellets  5

boolean motorON=false; //Para comprobar si está en marcha
boolean valvula=false; //control electroválvula 3 vías biomasa
//datos estadísticos y otras
long Mdatos[6] = {0,0,0,0,0,0}; //Matriz estadístias uso
//Matriz de arrays (*=pointers) de los nombres de los datos; No deberían tener más de 13 caracteres cada una (sumar 
const char* MnomDatos[]={"Hrs motor ON","Hrs Arduino","CTR RELE Motor","CTR VALVULA Fuego","CTR VALVULA Enfriar","KWh RENOVABLES"}; 
#define hrsMotorOn 0 //Para hacer referencia en código directamente
#define hrsArduino 1
#define ctrReleMotor 2
#define ctrValvulaFuego 3
#define ctrValvulaEnfriar 4
#define ctrKWh 5
double tempPrevia=0; //Temperatura cuando se activa el motor para el cálculo de Kwh ahorrados
double kwhAhorro=0; //Para cálculos
const double kwhGrado =  250 * (double(23)/double(859)); //Hacemos el cálculo de los kwh que suponen cada ºC que subimos la temp.; 1kWh son 859 kcal; 1 lt cuesta 23 Kcal

int activarValvEnfria=0; //Contador para activar la válvula de enfriado cada "horasValvEnfria" horas.

//Para ahorrar cálculos lo definimos como constante en esta parte del programa
const float K= 273.15; //Para pasar a grados Kelvin
const float e = 2.718281828459045; //Constante matemática 
//const float B = log(RPto2/RPto1)/(1/(TPto2+K)-(1/(TPto1+K))); //Valor Beta de tu termistor calculado de dos puntos
const float unodivr = 1/(resistor * pow(e,(-B/298.15))); //Con pow elevamos e al resultado

double T = 0; //Declaramos la variable Temperatura
int grados, decimas; //Para ponerle coma al resultado (en español)

float segundos=0; //Guardamos segundos funcionamiento (en sleepmode millis se para)
unsigned long millisAnterior=0; //Guardamos millis anterior
unsigned long segundosArduino=0; //Guardar horas funiconamiento Arduino
unsigned long segundosRefresca=0; //para refrescar pantalla
unsigned long segundosInicioMotor=0; //Guardar tiempo activado motor
unsigned long restoSegundosMotor=0; //Guardamos resto tiempo activación motor
unsigned long timeLED=0; //Tiempo funcionamiento LED placa
unsigned long segundosValvula=0; //Guardar momento activación válvula para desactivar después de 15 min.
byte error=0; //En caso de error será <> 0

int direccionInicial; //Dirección memoria EEPROM a partir de la cual se guardan los datos, asignado por la librería EEPROMex

void setup() {
  Serial.begin(9600);

//Descomentar las lÃ­neas siguientes para sincronizar tiempo por el Serial
//  if(Serial.available()) 
//  {
//     time_t t = processSyncMessage();
//     if(t >0)
//     {
//        RTC.set(t);   // set the RTC and the system time to the received value
//        setTime(t);          
//     }
//  }
//   digitalClockDisplay();  
//   delay(1000);
//}

  //sync internal time to the RTC
//  setSyncProvider(RTC.get);   // the function to get the time from the RTC
//  Serial.println();
//  if(timeStatus()!= timeSet) 
//     Serial.println("Unable to sync with the RTC");
//  else
//     Serial.println("RTC has set the system time");
//Wire.begin();
//RTC.begin();
//Si quitamos el comentario de la linea siguiente, se ajusta la hora y la fecha con la del ordenador
//RTC.adjust(DateTime(__DATE__, __TIME__));
  Serial.println("Arduino Working OK");

//Líneas de configuración del WatchDog Timer
/*** Setup the WDT ***/
  
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */ 
   //WDP3 - WDP2 - WPD1 - WDP0 - time
  // 0      0      0      0      16 ms
  // 0      0      0      1      32 ms
  // 0      0      1      0      64 ms
  // 0      0      1      1      0.125 s
  // 0      1      0      0      0.25 s
  // 0      1      0      1      0.5 s
  // 0      1      1      0      1.0 s
  // 0      1      1      1      2.0 s
  // 1      0      0      0      4.0 s
  // 1      0      0      1      8.0 s
  
  WDTCSR = 1<<WDP2 | 1<<WDP1 | 1<<WDP0; /* 2.0 seconds */
  //WDTCSR = 1<<WDP3; /* 4.0 seconds */
  //WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  
  Serial.println("Initialisation complete.");


//#ifdef USE_ADAFRUIT_SHIELD_PINOUT
//  Serial.println(F("Using Adafruit 2.8\" TFT Arduino Shield Pinout"));
//#else
//  Serial.println(F("Using Adafruit 2.8\" TFT Breakout Board Pinout"));
//#endif

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
  tft.reset();

  uint16_t identifier = tft.readID();

if(identifier == 0x9325) {
#ifdef DEBUG
    Serial.println("Found ILI9325 LCD driver");
#endif // DEBUG
  } else if(identifier == 0x9328) {
#ifdef DEBUG
    Serial.println("Found ILI9328 LCD driver");
#endif // DEBUG
  } else if(identifier == 0x7575) {
#ifdef DEBUG
    Serial.println("Found HX8347G LCD driver");
#endif // DEBUG
  } else if(identifier == 0x9341) {
#ifdef DEBUG
    Serial.println("Found ILI9341 LCD driver");
#endif // DEBUG
  } else if(identifier == 0x8357) {
#ifdef DEBUG
    Serial.println("Found HX8357D LCD driver");
#endif // DEBUG
  } else if(identifier == 0x0154) {
#ifdef DEBUG
    Serial.println("Found S6D0154 LCD driver");
#endif // DEBUG
    } else {
    #ifdef DEBUG
    Serial.print("Unknown LCD driver chip: ");
    Serial.println(identifier, HEX);
    Serial.print("I try use ILI9341 LCD driver ");
    Serial.println("If using the Adafruit 2.8\" TFT Arduino shield, the line:");
    Serial.println("  #define USE_ADAFRUIT_SHIELD_PINOUT");
    Serial.println("should appear in the library header (Adafruit_TFT.h).");
    Serial.println("If using the breakout board, it should NOT be #defined!");
    Serial.println("Also if using the breakout, double-check that all wiring");
    Serial.println("matches the tutorial.");
    #endif // DEBUG
    identifier = 0x9341;
  }


  tft.begin(identifier);

  //tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 320, 240, BLACK);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.print("    Arduino MEGA 2560");
  tft.setCursor(30, 100);
  tft.setTextColor(RED); 
  tft.setTextSize(3);
  tft.println("LCD driver chip: ");
  tft.setCursor(100, 150);
  tft.setTextColor(BLUE);
  tft.println(identifier, HEX);
  tft.setTextSize(1);
  tft.setCursor(90, 220);
  tft.println("Copyright 2016 Ringmaster v1.6");
  
  delay(6000);

  //Configuración TouchScreen
  #define MINPRESSURE 10
  #define MAXPRESSURE 1000
  
  // start reading from position memBase (address 0) of the EEPROM. Set maximumSize to EEPROMSizeUno 
  // Writes before membase or beyond EEPROMSizeUno will only give errors when _EEPROMEX_DEBUG is set
  EEPROM.setMemPool(20, EEPROMSizeMega);
  
  // Set maximum allowed writes to maxAllowedWrites. 
  // More writes will only give errors when _EEPROMEX_DEBUG is set
  EEPROM.setMaxAllowedWrites(10000);
  //Coger la dirección inicial siempre en el mismo momento y en el mismo orden
  direccionInicial = EEPROM.getAddress(4); //Primera dirección Long disponible a partir de la cual guardar

//Reseteamos a 0 la zona de EEPROM necesaria durante la primera ejecución
if (EEPROM.readLong(direccionInicial+45)!= 4011983) { //Si se ha reseteado ya, no lo volvemos a hacer
  for(int i=0; i<5; i++) {               
       EEPROM.writeLong(direccionInicial +(i*4),0);
        }
    EEPROM.writeLong(direccionInicial+45,4011983);
    Serial.println("Borrado de EEPROM terminado");
}

//Para guardar en EEPROM datos concretos
//EEPROM.writeLong(direccionInicial+(0),3000); //"Hrs motor ON"
//EEPROM.writeLong(direccionInicial+(4),945); //"Hrs Arduino"
//EEPROM.writeLong(direccionInicial+(8),50); //"CTR RELE Motor"
//EEPROM.writeLong(direccionInicial+(12),0); //"CTR VALVULA Fuego"
//EEPROM.writeLong(direccionInicial+(16),0); //"CTR VALVULA Enfriar"
//EEPROM.writeLong(direccionInicial+(20),0); //"KWh RENOVABLES"
        
//Coger valores almacenados EEPROM
for(int i=0; i<6; i++) {
    Mdatos[i]=EEPROM.readLong(direccionInicial+(i*4)); 
}
Serial.println("Datos recopilados");

pinMode(motorPIN, OUTPUT); digitalWrite(motorPIN,LOW);
pinMode(valvbioPIN, OUTPUT); digitalWrite(valvbioPIN,LOW);
pinMode(valvenfriaPIN, OUTPUT); digitalWrite(valvenfriaPIN,LOW);
pinMode(ledPIN, OUTPUT);
pinMode(zumbadorPIN, OUTPUT);
for(int i=primerSensor; i<numeroSensores; i++) {
  pinMode(i, INPUT);  
}

}

//----------------------------------PROGRAMA PRINCIPAL-------------------------
void loop() {
  millisAnterior=millis(); //Recogemos valor millis al inicio del programa para sumar luego la diferencia a "segundos"
  //time_t t = now(); //Cogemos la hora
//*******************************RECOGIDA Y CÁLCULO TEMPERATURAS SONDAS*********

//Borrar array
for(int i=0; i<numeroSensores; i++) 
      {               
         Msensores[i]=0;
      }  

//Recoge 5 veces para sacar medias
 for(int x=0; x<5; x++) 
    {
      for(int i=0; i<numeroSensores; i++) //termistores analógicos
      {               
         Msensores[i]=Msensores[i]+analogRead(i+primerSensor);
      }  
      delay(25); // espera 25ms entre lecturas
    }
Serial.println("Recogidas temperaturas");
//Sacar medias
for(int i=0; i<numeroSensores; i++) //termistores analógicos
      {               
         Msensores[i]=(Msensores[i]/5)+calibTemps[i];
      }  

Serial.println("Medias obtenidas");
//Comprobamos si las sondas están ok; si el valor es 0 (cero absoluto), la sonda estará desconectada
error=0; //reseteamos siempre al principio del bucle
for(int i=0; i<numeroSensores; i++) { //ermistores analógicos         
    if (Msensores[i]<50) {
      error++; //Avisamos con la alarma del error
      }
}  

//Convertimos los valores a la temperatura correcta
for(int i=0; i<numeroSensores; i++)
   {
    float v2 = (voltage*float(Msensores[i]))/1024.0f;  //Convertimos a voltios :)  
  
 // Parte 2: Calcular la resistencia con el valor de los voltios mediante la ecuación del divisor de voltaje
  //voltage = 4.83
  //R2 = 10000
  //R1 = Thermistor resistance
  //V2= v2
  //so V2=(R2*V)/(R1+R2)
  //and r1=((r2*v)/v2)-r2 <--final
  
  float r1a = (voltage*float(resistor))/v2;  
  float r1 =r1a - resistor;

  //Parte 3: Calcular la temperatura basandose en la ecuación Steinhart-Hart y la ecuación del valor Beta.
  // T=B/ln(r1/rinfinit)


  float T = B/log(r1*unodivr);
  Msensores[i]=T-273.15 + calibTemps[i]; //Convertimos a ºC, añadimos el valor de calibración y ya tenemos la temperatura
}  
Serial.println("Temperaturas convertidas");

//Activamos alarma si alguna temp es excesiva
for(int i=0; i<numeroSensores; i++) { //5 termistores analógicos               
     if (Msensores[i]>90) {
      error++; //Si se acumula algún error, haremos saltar la alarma en forma de zumbido
     }
 }  

//Guardamos valores anteriores
for(int i=0; i<numeroSensores; i++) { 
  MtempAnt[i]=Mtempsens[i];
}

//Pasamos los valores a la matriz en el orden que queremos en pantalla
Mtempsens[0]=Msensores[0];
Mtempsens[1]=Msensores[2];
Mtempsens[2]=Msensores[4];
Mtempsens[3]=Msensores[3];
Mtempsens[4]=Msensores[1];

//***************************COMPARACIÓN DE TEMPERATURAS SONDAS****************************
//Comprobamos biomasa, y actuamos sobre motor y electroválvulas en consecuencia
Serial.println("Comparando temperaturas"); //Atención; tengo en cuenta también Biomasa2 para activar por biomasa
//Si el fuego está encendido, activar motor y electroválvula circuito biomasa (PRIORITARIO)
if ((Mtempsens[biomasa1]-Mtempsens[retorno])>= DtFbiomasa or (Mtempsens[biomasa2]-Mtempsens[retorno])>= DtFbiomasa) {
    valvula=true;
    motorON=true;
  }
else { //Si lo anterior no se cumple, comprobar si ya no está caliente y si temperatura captador también está frío, apagar
  if ((Mtempsens[biomasa1]-Mtempsens[retorno])<= Dtobiomasa and (Mtempsens[biomasa2]-Mtempsens[retorno])<= Dtobiomasa and (Mtempsens[captador]-Mtempsens[retorno])< Dtosolar) {
    motorON=false;
  }
  //Si captador solar está caliente activamos motor
  if ((Mtempsens[captador]-Mtempsens[deposito])>= DtFsolar) { //Si diferencia temp. captador y depósito mayor que el delta, activar motor
    motorON=true;
    }
 }

//Serial.print("Temp deposito: ");
//Serial.println(Mtempsens[deposito]);
//Serial.print("Temp previa: ");
//Serial.println(tempPrevia);

//Dependiendo del resultado de las anteriores comprobaciones, actuamos sobre el motor y electroválvula
if (motorON==true) {
    if (digitalRead(motorPIN)==LOW) { //Se activa relé; contamos el número de activaciones
    Mdatos[ctrReleMotor]++;
    tempPrevia=Mtempsens[deposito]; //Guardamos temperatura actual deposito
    segundosInicioMotor=segundos; //Y activamos el contador de tiempo uso motor
  }
  digitalWrite(motorPIN,HIGH);
  }
else  {
    //Cada vez que paramos el motor, comprobamos diferencia temp. depósito y añadimos kwh ahorrados
    if (Mtempsens[deposito]>tempPrevia and digitalRead(motorPIN)==HIGH) {
      Serial.print("Valor kwh");
      Serial.println(kwhGrado);
      kwhAhorro=(Mtempsens[deposito]-tempPrevia)*kwhGrado;
      Serial.print("Ahorro");
      Serial.println(kwhAhorro);
      Mdatos[ctrKWh]=Mdatos[ctrKWh]+int(kwhAhorro); //Guardamos el dato
      segundosValvula=segundos; //Iniciamos cuenta atrás
    }
    digitalWrite(motorPIN,LOW);
  }

if (motorON==false and (segundos-segundosValvula)>=600) { //Si el motor sigue apagado 10 minutos, apagamos válvula tres vías.
      valvula=false;  //Para evitar accionamientos innecesarios en las bajadas de temp por recargas de leña
}

if (valvula==true) {
    if (digitalRead(valvbioPIN)==LOW) { //Se activa relé; contamos el número de activaciones
      Mdatos[ctrValvulaFuego]++;
        }
    digitalWrite(valvbioPIN, HIGH);
}
else  {
    digitalWrite(valvbioPIN,LOW);
  }

//Si la temperatura del depósito es excesiva, o éste no absorve suficiente calor del serpentín, activar el relé de enfriamiento (desvío a calefacción)
if (((Mtempsens[deposito]> DtFEnfriar) and motorON==true) or (motorON==true and Mtempsens[biomasa1]>80)) {
    if (digitalRead(valvenfriaPIN)==LOW) { //Se activa relé; contamos el número de activaciones
    MnomDatos[ctrValvulaEnfriar]++;
  }
  Serial.println("Activar válvula desvío circuito enfriado");
  digitalWrite(valvenfriaPIN,HIGH);
}
else { //Desactivar la válvula enfriado si la temp depósito ha bajado suficiente o el motor se ha parado (ya no hay más aporte de calor)
    if ((Mtempsens[biomasa1]<70 and Mtempsens[deposito]< DtoEnfriar) or motorON==false ) {//
    digitalWrite(valvenfriaPIN,LOW);
  }
}
  
Serial.println("Mostramos en pantalla");
// Y por ultimo lo mandamos a la pantalla LCD
//Cada hora actualizamos texto y estadísticas (y en los primeros segundos)
if ((segundos-segundosRefresca)>=3600 or millis()<10000) {
    segundosRefresca=segundos;
    tft.fillScreen(BLACK); //Es muy lento, tenerlo en cuenta
    tft.setTextSize(3);
    tft.setCursor(0,0);
    for(int i=0; i<numeroSensores; i++) { //5 termistores analógicos
          tft.setTextColor(YELLOW);
          tft.print(Mnombres[i]);  
          tft.println("     C");
      }
    //Escribimos estadísticas
    tft.setCursor(0,144);
    tft.setTextSize(2);
    tft.setTextColor(GREEN); //Indicamos ahorro Kwh y CO2 evitado
    tft.print(MnomDatos[ctrKWh]);
    tft.print(": ");
    tft.println(Mdatos[ctrKWh]);
//    tft.print("- Kg CO2: ");
//    tft.print(int(Mdatos[ctrKWh]*0.2016));
    tft.setTextColor(BLUE);
    for(int i=0; i<numeroSensores; i++) //Datos 
        {               
            tft.print(MnomDatos[i]);  
            tft.print(": ");
            tft.println(Mdatos[i]);
        }  
}

//Cada pocos seg. sólo actualizamos datos *********
tft.setTextSize(3);
for(int i=0; i<numeroSensores; i++) {//termistores analógicos
    //Borramos anterior
    tft.setTextColor(BLACK);
    tft.setCursor(162,i*24);
    if (Mtempsens[i]<=-270) {
      tft.print("ERROR");
    }
    else {
      grados=int(MtempAnt[i]);
        if (grados<10) {
          tft.print(" ");
        }
        tft.print(grados); //Grados
        tft.print(",");
        decimas=(MtempAnt[i]-grados)*10;
        tft.println(abs(decimas)); //décimas de grado
    }
    
    tft.setTextColor(YELLOW);
    tft.setCursor(162,i*24);
    if (Mtempsens[i]>-270) {
      if ((i==captador) || (i==biomasa1) || (i==biomasa2)) { //Si sonda captador o biomasa tienen mucha diferencia con respecto de depósito y retorno
        if ((Mtempsens[i]-Mtempsens[i+1])>difTemp) {
          tft.setTextColor(RED); //Ponemos la temperatura en rojo
          error++; //Activamos alarma
        }
      }
      grados=int(Mtempsens[i]);
      if (grados<10) {
        tft.print(" ");
      }
      tft.print(grados); //Grados
      tft.print(",");
      decimas=(Mtempsens[i]-grados)*10;
      tft.println(abs(decimas)); //décimas de grado
    }
    else { //La temp es incorrecta
      tft.setTextColor(RED);
      tft.println("ERROR");
      error++;
    }
}
//TEXTO DE AVISO
tft.setTextColor(BLACK);
tft.setCursor(0,120);
tft.print(TextoAnt);
//Ponemos lo nuevo
tft.setCursor(0,120);
tft.setTextSize(3);
if (motorON==true) {
  tft.setTextColor(RED);
  tft.print("MOTOR ON");
  TextoAnt="MOTOR ON";
}
else {
    tft.setTextColor(GREEN); //Indicamos ahorro KWh renovables
    tft.print("MOTOR OFF");
    TextoAnt="MOTOR OFF";
  }
if (digitalRead(valvenfriaPIN)==HIGH) { //Indicamos si está la válvula de enfriado activada
tft.setTextColor(WHITE);
tft.print(" ENFRIA");
TextoAnt=String(TextoAnt + " ENFRIA");
}
else {
  if (digitalRead(valvbioPIN)==HIGH) { //Electroválvula activada
  tft.setTextColor(WHITE);
  tft.print(" BIOMASA");
  TextoAnt=String(TextoAnt + " BIOMASA");
  }
}
  
//DEBUG *************
//Serial.print("Millis: ");
//Serial.println(segundos);
//Serial.print(F("MillisMotor: "));
//Serial.println(millisInicioMotor);


// _______________________________ GUARDAMOS TIEMPOS _______________________________________
//*** Si el motor ha estado activado, sumamos el tiempo y lo guardamos *******
//Estuvo encendido y acaba de apagarse; sumamos tiempo al contador de horas del motor y reseteamos
if (motorON==false and segundosInicioMotor>0) { 
    Mdatos[hrsMotorOn]= Mdatos[hrsMotorOn] + ((segundos+restoSegundosMotor-segundosInicioMotor)/3600);
    restoSegundosMotor=(segundos+restoSegundosMotor-segundosInicioMotor)-(int((segundos+restoSegundosMotor-segundosInicioMotor)/3600)*3600);
    segundosInicioMotor=0;
}
//DEBUG ***********
//Serial.print("HrsMotorOn: ");
//Serial.println(Mdatos[hrsMotorOn]);
//Serial.print("restoHrsMotor: ");
//Serial.println(restoSegundosMotor);

//*** Cada 3 horas salvamos a la EEPROM los datos que han cambiado (no hacerlo mas frecuente para prevenir el envejecimiento prematuro de la FLASH)
if ((segundos-segundosArduino)>=(3600*3)) { //Si ha pasado tres horas
      Serial.println("Guardamos horas Arduino y resto de datos en EEPROM");
      Mdatos[hrsArduino]=Mdatos[hrsArduino]+3; //hrs Arduino
      segundosArduino=segundos; //Actualizamos
      //Comprobamos si el contador de activado de válvula de desvío enfriamiento ha llegado a cero para activarla 5 segundos
      //Evitamos posibles agarrotamientos por falta de uso.
      activarValvEnfria++;
      if (activarValvEnfria>=horasValvEnfria) {
        activarValvEnfria=0;
        digitalWrite(valvenfriaPIN,HIGH);
        delay(5000);
        digitalWrite(valvenfriaPIN,LOW);
      }
	    //Salvamos los datos cambiados
	    for(int i=0; i<6; i++) { 
		    EEPROM.writeLong(direccionInicial+(i*4),Mdatos[i]);
        }
}
//_______________________________________________________________________

Serial.println("hacemos parpadear LED");
   //Hacemos parpadear al LED comprobando el tiempo desde la última activación
  if ((segundos-timeLED)>2) {
      digitalWrite(ledPIN,HIGH); //enciende LED indicando funcioanmiento
      timeLED=segundos;
  }
  else {
     digitalWrite(ledPIN,LOW); //apaga led
     }

if (error>0) { //Si hay algún problema, hacemos sonar la alarma intermitentemente
  for(int i=0; i<frecseg; i++) { //Durante el tiempo de espera
    digitalWrite(zumbadorPIN,HIGH);
    delay(300);
    digitalWrite(zumbadorPIN,LOW);
    delay(700);
    }
  }
else {
    digitalWrite(zumbadorPIN,LOW);
    Serial.println("Dormimos un poco... ZZZ");
    segundos=segundos + ((float(millis()-millisAnterior))/1000);
    for(int i=0; i<frecseg; i++) {
        enterSleep(); //delay(frecseg*1000);  //Bucle se ejecuta cada frecseg segundos
        segundos=segundos+2; //Actualizamos tiempo
    }
}

if (segundosInicioMotor>=segundos) { //Tenemos en cuenta si se ha desbordado segundos
   segundosInicioMotor=segundos; 
   segundosArduino=segundos;
   segundosValvula=segundos;
   segundosRefresca=0;
   millisAnterior=millis();
    } 


//código para depuración
//    Serial.print(F("Valor kwh"));
//    Serial.println(kwhGrado);
//    kwhAhorro=(Mtempsens[deposito]-Mtempsens[retorno])*kwhGrado; //para depurar
//    Serial.print(F("Ahorro"));
//    Serial.println(kwhAhorro);

//THE END
}


//RUTINAS
//Code from http://donalmorrissey.blogspot.com.es/2010/04/sleeping-arduino-part-5-wake-up-via.html
/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
 //Aquí el código que queremos se ejecute cuando el watchdog "despierta" al procesador

}



/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_SAVE for less power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}
