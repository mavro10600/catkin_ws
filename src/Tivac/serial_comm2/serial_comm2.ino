#include <Messenger.h>
#include <limits.h>
int counter;

Messenger Messenger_Handler=Messenger();

//Time update variables

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;

unsigned long milisNow;
unsigned long milisLastMsg;
bool timedOut=false;

int left_out=0;
int right_out=0;

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
   pinMode(LED_BUILTIN, OUTPUT);
   Messenger_Handler.attach(OnMssageCompleted);
  
}

void loop() 
{
unsigned long temp;
milisLastMsg=millis();
  temp = milisLastMsg-LastUpdateMillisecs;
  if (temp < 0)
    {
  temp = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
    }
    //Serial.print("temp: ");
    //Serial.print (temp);
    //Serial.print("\n");
  if (temp < 5000)
  {
  Read_From_Serial();
  Serial.print("a");
  Serial.print(" ");
  Serial.print(temp);
  Serial.print("\n");
  
  Update_Time();
  Update_Motors();  
  }
  if (temp >= 5000)
  {
    left_out=0;
    right_out=0;
  //Read_From_Serial();
  Serial.print("a");
  Serial.print(" ");
  Serial.print(temp);
  Serial.print("\n");
  
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
  
   while(Serial.available() > 0)
    {
      if(Serial.available()>0)
      {
         //milisLastMsg=millis();
       int data = Serial.read();
       Messenger_Handler.process(data);    
      }
           
    } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{
  
  char set_light_on[] = "5";
  char set_light_off[] = "0";
  char alive="a";  


  if(Messenger_Handler.checkString(set_light_on))
  {
     digitalWrite(LED_BUILTIN, HIGH);
     return; 
  }

  if(Messenger_Handler.checkString(set_light_off))
  {
     digitalWrite(LED_BUILTIN, LOW);
     return; 
  }
}

void Update_Motors()
{
  if (left_out < 9)
  left_out+=1;
  else
  left_out = 0;
  Serial.print("r");
  Serial.print (" ");
  Serial.print(left_out);
  Serial.print (" ");
  Serial.print(right_out);
  Serial.print("\n");
  
  delay(50);
  

}

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
/*
  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
  */
}

