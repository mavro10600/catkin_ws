/*
 
 Circuit:
 SCP1000 sensor attached to pins 6, 7, 10 - 13:
 DRDY: pin 6
 CSB: pin 8
 MOSI: pin 14/15
 MISO: pin 15/14 * need to level convert this
 SCK: pin 7
  
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include <Wire.h>

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int chipSelectPin2 = PB_0;//creo que este pi no es necesario en el as5043
const int chipSelectPin1 = PB_5; //pin de seleccion de esclavo :)


void setup() {
  Serial.begin(115200);

  // start the SPI library:
  SPI.begin();
  Wire.begin();

  // initialize the  data ready and chip select pins:
    pinMode(chipSelectPin1, OUTPUT);
  pinMode(chipSelectPin2, OUTPUT);
}

void loop() {

  // don't do anything until the data ready pin is high:
  
    //Read the temperature data
    int angle1 = readRegister(2,chipSelectPin1);

    // convert the temperature to celsius and display it:
    int realAngle1 = angle1>>6 ;
    Serial.print("Angle1=");
    Serial.println(angle1);

    
    //Read the temperature data
  //  int angle2 = readRegister(2,chipSelectPin2);

    // convert the temperature to celsius and display it:
//    int realAngle2 = angle2>>6 ;
//    Serial.print("Angle2=");
//    Serial.println(angle2);
//   Serial.print("Time=");
//    Serial.println(millis());

delay(100);
    
}

//Read from or write to register from the SCP1000:
unsigned int readRegister(int bytesToRead , int pin) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  //Serial.print(thisRegister, BIN);
 // Serial.print("\t");
  // SCP1000 expects the register name in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  
  // now combine the address and the command into one byte
  //byte dataToSend = thisRegister & READ;
  //Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(pin, LOW);
  // send the device the register you want to read:
  //SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
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


//Sends a write command to SCP1000
/*
void writeRegister(byte thisRegister, byte thisValue) {

  // SCP1000 expects the register address in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister | WRITE;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}

*/
