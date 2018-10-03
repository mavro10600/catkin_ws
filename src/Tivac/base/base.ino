#include <Messenger.h>
#include <limits.h>
//#include "RoboClaw.h"
#include "OSMC.h"
#include "AS5043.h"
#include "Encoder.h"
#include <SPI.h>
#include <Wire.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Estructura del programa
Define los objetos de lo OSMC, pines
Define los objetos asociados a los encoders, pines de prog, DO y CLK,
usamos rutinas de forma digital 
Define los encoders asigna pines analogicos, deberia de asignar tambien pines de seleccion clk y do
Defino variables de control de los motores
Defino variables de lectura de encoders
Defino roboclaws
SETUP //aqui quizas lo mas importante es fijarse que se inicien los objetos OSMC y los objetos de AS5043
definidos anteriormente
Loop
  Read from serial
  On_mssg_Completed
    llama a la siguiente funcion para leer los datos de entrafda desde ros psra asignar valores de pwm 
    a los osmc y roboclaw
  Set speed
    Descompone la cadena de entrada de datos en los valores a asignar a las variales de control de motores
  Reset
    Funcion de reset
  Update encoders
    LLamamos a la funcion read de cada objeto tipo encoder declarado, además se asignan pines de 
     lectura analógica para leer el voltaje de las baterias y la corriente de los motorees de traccion
  Update motors
    Mandamos los valores obtenidos en Set speed a las instrucciones de movimiento de los motores
  Update time
    Funcion de actualizacion de tiempo de ejecución

*//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
//DEfinimos los pines de los encoders
/*
#define pindoIzq PA_6
#define pinclkIzq PA_7
#define pincsnIzq PE_3 

#define pindoDer PF_1
#define pinclkDer PF_3
#define pincsnDer PF_2 

#define pindo1 PD_2
#define pinclk1 PD_0
#define pincsn1 PD_1 

#define pindo2 PE_2
#define pinclk2 PD_3
#define pincsn2 PE_1 

#define pindo3 PC_4
#define pinclk3 PB_3
#define pincsn3 PC_5 

#define pindo4 PA_4
#define pinclk4 PA_3
#define pincsn4 PA_2 
*/
//Redefinimos los pines esclavo de la comunicación SPI

#define pincsn1 PA_5
#define pincsn2 PE_5
#define pincsn3 PE_4
#define pincsn4 PB_1
#define pincsn5 PB_0
#define pincsn6 PB_5



//Definicion de los endstop
/*
#define endstop1 PE_5
#define endstop2 PF_0
#define endstop3 PF_4
#define endstop4 PE_0
*/
//Nueva placa, redefinimos los finales de carrera

#define endstop1 PE_0
#define endstop2 PF_0
#define endstop3 PA_3
#define endstop4 PA_2


///////////////////////////////////////////////////////////////////////////////////////////////////
//(pinpwm1,pinpwm2,umbral, maxpwmsense)
//OSMCClass LEFT(5,3,2,1,127);
//OSMCClass RIGHT(9,6,4,1,127);

//OSMCClass LEFT(PB_5,PB_0,PE_4,1,127);
//OSMCClass RIGHT(PB_1,PB_4,PA_5,1,127);

//OSMCClass LEFT(PB_0,PB_5,PE_4,1,127);
//OSMCClass RIGHT(PB_4,PB_1,PA_5,1,127);

// Redefinimos para la Nueva placa 
OSMCClass LEFT(PD_0,PD_1,PE_2,1,127);
OSMCClass RIGHT(PD_3,PD_2,PE_1,1,127);

//(clk,dO,pROG)
//(AS5043,CSn,input_min,input_max,output_max_abs_sense)
            
////////////////////////TODO agrgar los sensores de los flippers

Messenger Messenger_Handler=Messenger();
#define RESET_PIN PB_2

////////////////////////////
//Time update variables

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;

///////////////////////////////////////////////////////////////
// Valores de control a los motores
int left_out=0;
int right_out=0;
int flipper1_out=64;
int flipper2_out=0;
int flipper3_out=64;
int flipper4_out=0;

///////////////////////////////////////////////////////////////////
//Variables de los encoders
int stat_complete=0;

int vueltasIzq;
unsigned int last_lecIzq;
boolean statIzq;

int vueltasDer;
unsigned int last_lecDer;
boolean statDer;

int vueltas1;
unsigned int last_lec1;
boolean stat1;
boolean fl1_stat;

int vueltas2;
unsigned int last_lec2;
boolean stat2;
boolean fl2_stat;

int vueltas3;
unsigned int last_lec3;
boolean stat3;
boolean fl3_stat;

int vueltas4;
unsigned int last_lec4;
boolean stat4;
boolean fl4_stat;

int left_lec=0;
int right_lec=0;
int flip1_lec=0;
int flip2_lec=0;
int flip3_lec=0;
int flip4_lec=0;

/////////////////////////////////////////////////////////////////////
//Agregar variables de la corriente de los motores y 
//de los finales de carrera, son dos de los motores y dos finales de carrera



bool endstp1;
bool endstp1prev;
bool endstp2;
bool endstp2prev;
bool endstp3;
bool endstp3prev;
bool endstp4;
bool endstp4prev;

//////////////////////////////////////////////////////////////////////
//Roboclaws
//Uncomment if Using Hardware Serial port
//RoboClaw roboclaw(&Serial2,10000);
//RoboClaw rc(&Serial3,10000);

#define address 0x80


void setup() {
Serial.begin(115200);//cuando se ve en el ide de arduino
SPI.begin();
Wire.begin();

  //roboclaw.begin(38400);
  Serial2.begin(38400);
  //rc.begin(38400);
  Serial3.begin(38400);
  SetupEncoders();
  
  SetupMotors();
  
  SetupReset();

  Messenger_Handler.attach(OnMssageCompleted);

  SetupEndstop();
}

void SetupEncoders()
{
pinMode(pincsn1,OUTPUT);  
pinMode(pincsn2,OUTPUT);  
pinMode(pincsn3,OUTPUT);  
pinMode(pincsn4,OUTPUT);  
pinMode(pincsn5,OUTPUT);  
pinMode(pincsn6,OUTPUT);  
}

void SetupMotors()
{
  //Para los OSMC
  pinMode(PE_2, OUTPUT);
  pinMode(PE_1, OUTPUT);
  LEFT.begin();
  RIGHT.begin();
}

void SetupReset()
{
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  ///Conectar el pin de reset al pin reset de la placa
  digitalWrite(RESET_PIN,LOW);
}

void SetupEndstop()
{

  pinMode(endstop1,INPUT_PULLUP);

  attachInterrupt(endstop1,end1,CHANGE);

  
  pinMode(endstop2,INPUT_PULLUP);
  attachInterrupt(endstop2,end2,CHANGE);

  
  pinMode(endstop3,INPUT_PULLUP);


  attachInterrupt(endstop3,end3,CHANGE);
  
  
  
  pinMode(endstop4,INPUT_PULLUP);

  attachInterrupt(endstop4,end4,CHANGE);



if (digitalRead(endstop1)==HIGH){endstp1prev==true;}
if (digitalRead(endstop1)==LOW){endstp1prev==false;}

if (digitalRead(endstop2)==HIGH){endstp2prev==true;}
if (digitalRead(endstop2)==LOW){endstp2prev==false;}

if (digitalRead(endstop3)==HIGH){endstp3prev==true;}
if (digitalRead(endstop3)==LOW){endstp3prev==false;}

if (digitalRead(endstop4)==HIGH){endstp4prev==true;}
if (digitalRead(endstop4)==LOW){endstp4prev==false;}

}

 


void loop() {
    
  Read_From_Serial();
  Update_Time();
  Update_Motors();  
  Update_Encoders();
  Update_Endstops();
  delay(20);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
  
   while(Serial.available() > 0)
    {
      if(Serial.available()>0)
      {
       int data = Serial.read();
       Messenger_Handler.process(data);    
      }

    } 
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  char set_flippers[]="f";
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Done"); 
     Reset();
    
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    
     //This will set the speed
     Set_Speed();
     return; 
  }

  if(Messenger_Handler.checkString(set_flippers))
  {
    
     //This will set the speed
     Set_Flippers();
     return; 
  }
  
  else 
  {
    left_out=0;
    right_out=0;
    //TODO
    //cambiar a nuevos valores de paro
    flipper1_out=64;
    flipper2_out=0;
    flipper3_out=64;
    flipper4_out=0;
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{    
    left_out=Messenger_Handler.readLong();
    right_out=Messenger_Handler.readLong();
    /*flipper1_out=Messenger_Handler.readLong();
    flipper2_out=Messenger_Handler.readLong();
    flipper3_out=Messenger_Handler.readLong();
    flipper4_out=Messenger_Handler.readLong();
*/
}

//////////////////////////////////////////////////////////////////////////
//Set_flippers
void Set_Flippers()
{    
    /*left_out=Messenger_Handler.readLong();
    right_out=Messenger_Handler.readLong();
    */
    flipper1_out=Messenger_Handler.readLong();
    flipper2_out=Messenger_Handler.readLong();
    flipper3_out=Messenger_Handler.readLong();
    flipper4_out=Messenger_Handler.readLong();

    //TODO 
    //cambiar aqui tambien el mapeo
if(flipper1_out>0){fl1_stat=true;}
if(flipper1_out<0){fl1_stat=false;}
if(flipper2_out>0){fl2_stat=true;}
if(flipper2_out<0){fl2_stat=false;}
if(flipper3_out>0){fl3_stat=true;}
if(flipper3_out<0){fl3_stat=false;}
if(flipper4_out>0){fl4_stat=true;}    
if(flipper4_out<0){fl4_stat=false;}
}




////////////////////////////////////////////////////////////////////////
//funcion de reset
void Reset()
{
  
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW);
}

///////////////////////////////////////////////////////////////
//funcion de leer los encoders
void Update_Encoders()
{
  /*
 left_lec=encoder_digital(pindoIzq,pinclkIzq,pincsnIzq,&last_lecIzq,&vueltasIzq,&statIzq);
 right_lec=encoder_digital(pindoDer,pinclkDer,pincsnDer,&last_lecDer,&vueltasDer,&statDer);
 flip1_lec=encoder_digital(pindo1,pinclk1,pincsn1,&last_lec1,&vueltas1,&stat1);
 flip2_lec=encoder_digital(pindo2,pinclk2,pincsn2,&last_lec2,&vueltas2,&stat2);
 flip3_lec=encoder_digital(pindo3,pinclk3,pincsn3,&last_lec3,&vueltas3,&stat3);
 flip4_lec=encoder_digital(pindo4,pinclk4,pincsn4,&last_lec4,&vueltas4,&stat4);
*/

left_lec=encoder(2,pincsn1);
right_lec=encoder(2,pincsn2);

flip1_lec=encoder(2,pincsn3);
flip2_lec=encoder(2,pincsn4);
flip3_lec=encoder(2,pincsn5);
flip4_lec=encoder(2,pincsn6);

 Serial.print("e");
  Serial.print("\t");
  Serial.print(left_lec>>6);
  Serial.print("\t");
  Serial.print(right_lec>>6);
  Serial.print("\t");
  Serial.print(flip1_lec>>6);
  Serial.print("\t");
  Serial.print(flip2_lec>>6);
  Serial.print("\t");
  Serial.print(flip3_lec>>6);
  Serial.print("\t");
  Serial.print(flip4_lec>>6);
  Serial.print("\n");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update motors function

void Update_Motors()
{
 //introducir aqui la lectura de corriente
 //bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2);
 //y una instruccion de seguridad, para enviar un cero a los motores si pasan cierta coriiente
 //También podriamos enviar una cadena con los valores de corriente :)
//digitalWrite(PE_1,LOW);
//digitalWrite(PE_2,LOW); 
LEFT.write(left_out);
RIGHT.write(right_out);
  Serial.print("d");
  Serial.print("\t");
  Serial.print(left_out);
  Serial.print("\t");
  Serial.print(right_out);
  Serial.print("\n");

//////////////////////////////////////////////////////////////////////////
//TODO cambiar el mapeo en la roboclaw
/*The RoboClaw standard serial is setup to control both motors with one byte sized command
character. Since a byte can be any value from 0 to 255(or -128 to 127) the control of each motor
is split. 1 to 127 controls channel 1 and 128 to 255(or -1 to -127) controls channel 2. Command
value 0 will stop both channels. Any other values will control speed and direction of the specific
channel.
*/

if (endstp1==true)
{
  //roboclaw.ForwardBackwardM1(address,64);
  if(fl1_stat==false){if(flipper1_out>0){Serial2.write(64+flipper1_out);}else {Serial2.write(64);}}
  if(fl1_stat==true){if(flipper1_out<0){Serial2.write(64);}else {Serial2.write(64+flipper1_out);}}
}

if (endstp1==false)
Serial2.write(64+flipper1_out);

////////////////////////////////////////////////////////////////////////////

if (endstp2==true)
//roboclaw.ForwardBackwardM2(address,64);
{
  //roboclaw.ForwardBackwardM1(address,64);
  if(fl2_stat==true){if(flipper2_out<0){Serial2.write(192+flipper2_out);}else {Serial2.write(192);}}
  if(fl2_stat==false){if(flipper2_out>0){Serial2.write(192);}else {Serial2.write(192+flipper2_out);}}
}

if (endstp2==false)
Serial2.write(192+flipper2_out);

/////////////////////////////////////////////////////////////////////////

if (endstp3==true)
//rc.ForwardBackwardM1(address,64);
{
  //roboclaw.ForwardBackwardM1(address,64);
  if(fl3_stat==true){if(flipper3_out>0){Serial3.write(64+flipper3_out);}else {Serial3.write(64);}}
  if(fl3_stat==false){if(flipper3_out<0){Serial3.write(64);}else {Serial3.write(64+flipper3_out);}}
}

if (endstp3==false)
Serial3.write(64+flipper3_out);

//////////////////////////////////////////////////////////////////
//
if (endstp4==true)
//rc.ForwardBackwardM2(address,64);
{
  //roboclaw.ForwardBackwardM1(address,64);
  if(fl4_stat==true){if(flipper4_out>0){Serial3.write(192+flipper4_out);}else {Serial3.write(192);}}
  if(fl4_stat==false){if(flipper4_out<0){Serial3.write(192);}else {Serial3.write(192+flipper4_out);}}
}

if (endstp4==false)
Serial3.write(192+flipper4_out);

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function

void Update_Time()
{
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
  MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;

    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Funcion para leer cada encoder pasando como valores de referencia 


unsigned int encoder(int bytesToRead, int pin)
{
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  digitalWrite(pin, LOW);
  result = SPI.transfer(0x00);
  bytesToRead--;
  if (bytesToRead > 0) 
  {
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(pin, HIGH);
  // return the result:
  return(result);
  
  
  
}





void Update_Endstops()
{
  int endtemp1,endtemp2,endtemp3,endtemp4;

  if(endstp1){endtemp1=1;} else {endtemp1=0;}
  if(endstp2){endtemp2=1;} else {endtemp2=0;}
  if(endstp3){endtemp3=1;} else {endtemp3=0;}
  if(endstp4){endtemp4=1;} else {endtemp4=0;}
  
  
  Serial.print("n");
  Serial.print("\t");
  Serial.print(endtemp1);
  Serial.print("\t");
  Serial.print(endtemp2);
  Serial.print("\t");
  Serial.print(endtemp3);
  Serial.print("\t");
  Serial.print(endtemp4);
  Serial.print("\n");  
}



void end1()
{
  if(digitalRead(endstop1)==LOW)
  {delay (1);
    if(digitalRead(endstop1)==LOW)
  {endstp1=true;  endstp1prev=!endstp1prev;}}

  if (digitalRead(endstop1)==HIGH ){
  delay (1);
  if (digitalRead(endstop1)==HIGH)
  {endstp1=false;}
  }//else {endstp1=false; endstp1prev=false;}
}

void end2()
{
  if(digitalRead(endstop2)==LOW)
  {delay (1);
  if(digitalRead(endstop2)==LOW)
  {endstp2=true;}
  }
  if (digitalRead(endstop2)==HIGH)
  {
    delay (1);
  if (digitalRead(endstop2)==HIGH)
  {endstp2=false; }
  }
  
  //else {endstp1=false; endstp1prev=false;}
}

void end3()
{
  if(digitalRead(endstop3)==LOW)
  {delay (1);
  if(digitalRead(endstop3)==LOW )
  {
  endstp3=true;
  }}
  
  if (digitalRead(endstop3)==HIGH )
  {delay (1);
  if (digitalRead(endstop3)==HIGH )
  {endstp3=false;}
  }//else {endstp1=false; endstp1prev=false;}

}

void end4()
{
   if(digitalRead(endstop4)==LOW )
   {delay (1);
   if(digitalRead(endstop4)==LOW )
  {endstp4=true;}
  }

  if (digitalRead(endstop4)==HIGH )
  {
    delay (1);
  if (digitalRead(endstop4)==HIGH )
  {endstp4=false; }}
  
  //else {endstp1=false; endstp1prev=false;}
}




void set_status()
{
  if(stat1)  bitWrite(stat_complete,0,1); else bitWrite(stat_complete,0,0);
  if(stat2)  bitWrite(stat_complete,1,1); else bitWrite(stat_complete,1,0);
  if(stat3)  bitWrite(stat_complete,2,1); else bitWrite(stat_complete,2,0);
  if(stat4)  bitWrite(stat_complete,3,1); else bitWrite(stat_complete,3,0);
  if(statIzq)  bitWrite(stat_complete,5,1); else bitWrite(stat_complete,5,0);
  if(statDer)  bitWrite(stat_complete,6,1); else bitWrite(stat_complete,6,0);
}



