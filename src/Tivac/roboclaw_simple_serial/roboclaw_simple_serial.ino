
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);//cuando se ve en el ide de arduino

  //roboclaw.begin(38400);
  Serial2.begin(38400);
  //rc.begin(38400);
  Serial3.begin(38400);

}

void loop() {
  // put your main code here, to run repeatedly: 
Serial2.write(64);
Serial2.write(50);
delay(100);
Serial2.write(78);
delay(100);
Serial2.write(64);

Serial2.write(192);
Serial2.write(178);
delay(100);
Serial2.write(206);
delay(100);
Serial2.write(192);

  
}
