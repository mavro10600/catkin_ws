/*
  Arduino________COZIR Sensor
   GND ------------------ 1 (gnd)
   3.3v------------------- 3 (Vcc)  
    12 -------------------- 5 (Rx)
    13 -------------------- 7 (Tx)
*/
//#include <SoftwareSerial.h>

//SoftwareSerial mySerial(12, 13); // RX, TX pins on Ardunio

int co2 =0;
double multiplier = 5;// 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM
uint8_t buffer[25];
uint8_t ind =0;
uint8_t indice =0;

int fill_buffer();  // function prototypes here
int format_output();

void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");
  Serial.println("AN128_ardunio_cozir CO2 Demonstration code 11/22/2016\n\n"); 
  Serial2.begin(9600); // Start serial communications with sensor
  Serial2.println("K 0");  // Set Command mode
  Serial2.println("K 2");  // set polling mode
  Serial2.readString();
  
}

void loop() {
  Serial2.println("Z");
  if(Serial2.available()){
    if(Serial2.read() == 'Z'){
      Serial2.read();
      Serial.println(Serial2.readStringUntil('Z').toInt());
      Serial2.readString(); 
    }
  }
  //delay(500);
  
//fill_buffer();
//format_output();
   
}



