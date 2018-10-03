/*
Descripcion Este programa describe como usar el dynamixel 
usando la biblioteca proprocionada por savage electronics
*/

#include <DynamixelSerial.h>


int Direction_Pin=PA_2;

int Position,Voltage,Temperature;
void setup() {
  Dynamixel.begin(1000000,Direction_Pin);
  Serial.begin(115200);

}

void loop() {
  Serial.print("tiempo: ");
  Serial.print(millis());
  Serial.print("-");
  
  Position=Dynamixel.readPosition(1);
  Voltage=Dynamixel.readVoltage(1);
  Temperature=Dynamixel.readTemperature(1);
  //Dynamixel.move(1,random(100,600));
   Serial.print("tiempo2: ");
  Serial.print(millis());
  Serial.print("-");
  Serial.print(Position,DEC);
  Serial.print("-");
  Serial.print(Voltage,DEC);
  Serial.print("-");
  Serial.print(Temperature,DEC);
  Serial.print("\n");
  
}
