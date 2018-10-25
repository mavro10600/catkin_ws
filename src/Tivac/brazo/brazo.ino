#include <Messenger.h>
#include <limits.h>
//#include "RoboClaw.h"
#include "OSMC.h"
//#include "AS5043.h"
//#include "Encoder.h"
#include <SPI.h>
#include <Wire.h>
#include "Talon.h"
#include <Servo.h>
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

#define pincsn1 PA_3
#define pincsn2 PE_3
#define pincsn3 PE_2



//Definicion de los endstop
/*
#define endstop1 PE_5
#define endstop2 PF_0
#define endstop3 PF_4
#define endstop4 PE_0
*/
//Nueva placa, redefinimos los finales de carrera

#define endstopBase1 PF_1
#define endstopBase2 PA_7
#define endstopShoulder1 PA_6
#define endstopShoulder2 PA_5
#define endstopElbow1 PB_4
#define endstopElbow2 PE_5

#define pinCytronDir PC_5
#define pinCytronPwm PC_4

///////////////////////////////////////////////////////////////////////////////////////////////////
//(pinpwm1,pinpwm2,umbral, maxpwmsense)
//OSMCClass LEFT(5,3,2,1,127);
//OSMCClass RIGHT(9,6,4,1,127);

//OSMCClass LEFT(PB_5,PB_0,PE_4,1,127);
//OSMCClass RIGHT(PB_1,PB_4,PA_5,1,127);

//OSMCClass LEFT(PB_0,PB_5,PE_4,1,127);
//OSMCClass RIGHT(PB_4,PB_1,PA_5,1,127);

Servo base;
//umbral,max
TalonClass BASE(2,50);

// Redefinimos para la Nueva placa 

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
int shoulder_out=0;
int elbow_out=0;
int base_out=0;
int roll_out=0;
int pitch_out=0;
int yaw_out=0;
///////////////////////////////////////////////////////////////////
//Variables de los encoders
int stat_complete=0;


int vueltas0;
unsigned int last_lec0;
boolean stat0;
boolean base_stat;

int vueltas1;
unsigned int last_lec1;
boolean stat1;
boolean shoulder_stat;

int vueltas2;
unsigned int last_lec2;
boolean stat2;
boolean elbow_stat;

int base_lec=0;
int shoulder_lec=0;
int elbow_lec=0;

/////////////////////////////////////////////////////////////////////
//Agregar variables de la corriente de los motores y 
//de los finales de carrera, son dos de los motores y dos finales de carrera



bool endstp_base;
bool endstp_baseprev;
bool endstp_shoulder;
bool endstp_shoulderprev;
bool endstp_elbow;
bool endstp_elbowprev;

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
  //SetupEncoders();
  
  SetupMotors();
  
  SetupReset();

  Messenger_Handler.attach(OnMssageCompleted);

//  SetupEndstop();
}

void SetupEncoders()
{
pinMode(pincsn1,OUTPUT);  
pinMode(pincsn2,OUTPUT);  
pinMode(pincsn3,OUTPUT);  
}

void SetupMotors()
{
  //Para los OSMC
 base.attach(PB_3);

  pinMode(pinCytronPwm,OUTPUT);
  pinMode(pinCytronDir,OUTPUT);
  digitalWrite(pinCytronDir,LOW);
  digitalWrite(pinCytronPwm,LOW);
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

  pinMode(endstopBase1,INPUT_PULLUP);

  attachInterrupt(endstopBase1,end1,CHANGE);

  
  pinMode(endstopShoulder1,INPUT_PULLUP);
  attachInterrupt(endstopShoulder1,end2,CHANGE);

  
  pinMode(endstopElbow1,INPUT_PULLUP);


  attachInterrupt(endstopElbow1,end3,CHANGE);
  
  
  

if (digitalRead(endstopBase1)==HIGH){endstp_baseprev==true;}
if (digitalRead(endstopBase1)==LOW){endstp_baseprev==false;}

if (digitalRead(endstopShoulder1)==HIGH){endstp_shoulderprev==true;}
if (digitalRead(endstopShoulder1)==LOW){endstp_shoulderprev==false;}

if (digitalRead(endstopElbow1)==HIGH){endstp_elbowprev==true;}
if (digitalRead(endstopElbow1)==LOW){endstp_elbowprev==false;}

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
  
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Done"); 
     Reset();
    
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    
     //This will set the speed
     //Serial.print("Set_speed");
     Set_Speed();
     return; 
  }

    /*
  else 
  {
      //TODO
    //cambiar a nuevos valores de paro
    shoulder_out=64;
    elbow_out=0;
    return;
  
  }
*/
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed

void Set_Speed()
{    
    /*left_out=Messenger_Handler.readLong();
    right_out=Messenger_Handler.readLong();
    */
    base_out=Messenger_Handler.readLong();
    shoulder_out=Messenger_Handler.readLong();
    elbow_out=Messenger_Handler.readLong();
    roll_out=Messenger_Handler.readLong();
    pitch_out=Messenger_Handler.readLong();
    yaw_out=Messenger_Handler.readLong();
    //Serial.print("Set_speed_funct");
    
     //TODO 
    //cambiar aqui tambien el mapeo

if(base_out>0){base_stat=true;}
if(base_out<0){base_stat=false;}    
if(shoulder_out>0){shoulder_stat=true;}
if(shoulder_out<0){shoulder_stat=false;}
if(elbow_out>0){elbow_stat=true;}
if(elbow_out<0){elbow_stat=false;}

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
shoulder_lec=0;
elbow_lec=0;


//shoulder_lec=encoder(2,pincsn1);
//elbow_lec=encoder(2,pincsn2);

 Serial.print("e");
  Serial.print("\t");
  Serial.print(base_out);
  Serial.print("\t");
  Serial.print(shoulder_out);
  Serial.print("\t");
  Serial.print(elbow_out);
  Serial.print("\t");
  Serial.print(roll_out);
  Serial.print("\t");
  Serial.print(pitch_out);
  Serial.print("\t");
  Serial.print(yaw_out);
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
//////////////////////////////////////////////////////////////////////////
//TODO cambiar el mapeo en la roboclaw
/*The RoboClaw standard serial is setup to control both motors with one byte sized command
character. Since a byte can be any value from 0 to 255(or -128 to 127) the control of each motor
is split. 1 to 127 controls channel 1 and 128 to 255(or -1 to -127) controls channel 2. Command
value 0 will stop both channels. Any other values will control speed and direction of the specific
channel.
*/
base.writeMicroseconds(1500+base_out);
//Serial.print("Set_motors");
/*if (endstp_shoulder==true)
{
  //roboclaw.ForwardBackwardM1(address,64);
  if(shoulder_stat==false){if(shoulder_out>0){Serial2.write(64+shoulder_out);}else {Serial2.write(64);}}
  if(shoulder_stat==true){if(shoulder_out<0){Serial2.write(64);}else {Serial2.write(64+shoulder_out);}}
}*/

//if (endstp_shoulder==false)
Serial2.write(64+shoulder_out);

////////////////////////////////////////////////////////////////////////////
/*
if (endstp_elbow==true)
//roboclaw.ForwardBackwardM2(address,64);
{
  //roboclaw.ForwardBackwardM1(address,64);
  if(elbow_stat==true){if(elbow_out<0){Serial2.write(192+elbow_out);}else {Serial2.write(192);}}
  if(elbow_stat==false){if(elbow_out>0){Serial2.write(192);}else {Serial2.write(192+elbow_out);}}
}
*/
//if (endstp_elbow==false)
Serial2.write(192+elbow_out);

/////////////////////////////////////////////////////////////////////////

Serial3.write(64+roll_out);

/////////////////////////////////////////////////////////////////////////

Serial3.write(192+yaw_out);

/////////////////////////////////////////////////////////////////////////

int pitch_temp;
if(pitch_out>0){digitalWrite(pinCytronDir, HIGH); pitch_temp=map(pitch_out,0,500,0,127);}
if(pitch_out<0){digitalWrite(pinCytronDir, LOW); pitch_temp=map(-pitch_out,0,500,0,127);}
analogWrite(pinCytronPwm,pitch_temp);

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
  int endtemp1,endtemp2,endtemp3;

  if(endstp_base){endtemp1=1;} else {endtemp1=0;}
  if(endstp_shoulder){endtemp2=1;} else {endtemp2=0;}
  if(endstp_elbow){endtemp3=1;} else {endtemp3=0;}
  
  
  
  Serial.print("n");
  Serial.print("\t");
  Serial.print(endtemp1);
  Serial.print("\t");
  Serial.print(endtemp2);
  Serial.print("\t");
  Serial.print(endtemp3);
  Serial.print("\n");  
}



void end1()
{
  if(digitalRead(endstopBase1)==LOW)
  {delay (1);
    if(digitalRead(endstopBase1)==LOW)
  {endstp_base=true;  endstp_baseprev=!endstp_baseprev;}}

  if (digitalRead(endstopBase1)==HIGH ){
  delay (1);
  if (digitalRead(endstopBase1)==HIGH)
  {endstp_base=false;}
  }//else {endstp1=false; endstp1prev=false;}
}

void end2()
{
  if(digitalRead(endstopShoulder1)==LOW)
  {delay (1);
  if(digitalRead(endstopShoulder1)==LOW)
  {endstp_shoulder=true;}
  }
  if (digitalRead(endstopShoulder1)==HIGH)
  {
    delay (1);
  if (digitalRead(endstopShoulder1)==HIGH)
  {endstp_shoulder=false; }
  }
  
  //else {endstp1=false; endstp1prev=false;}
}

void end3()
{
  if(digitalRead(endstopElbow1)==LOW)
  {delay (1);
  if(digitalRead(endstopElbow1)==LOW )
  {
  endstp_elbow=true;
  }}
  
  if (digitalRead(endstopElbow1)==HIGH )
  {delay (1);
  if (digitalRead(endstopElbow1)==HIGH )
  {endstp_elbow=false;}
  }//else {endstp1=false; endstp1prev=false;}

}



void set_status()
{
  if(stat1)  bitWrite(stat_complete,0,1); else bitWrite(stat_complete,0,0);
  if(stat2)  bitWrite(stat_complete,1,1); else bitWrite(stat_complete,1,0);


}



