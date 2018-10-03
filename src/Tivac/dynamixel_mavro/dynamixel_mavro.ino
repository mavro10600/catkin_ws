#define AX_START                    255
#define AX_POS_LENGTH               4
#define AX_READ_DATA                2
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_READ_DATA                2
#define AX_BYTE_READ_POS            2
#define TX_DELAY_TIME        400 
#define TIME_OUT                    10

int Direction_Pin=PA_2;

void setup() {
  pinMode  (Direction_Pin,OUTPUT);
  // put your setup code here, to run once:
Serial3.begin(1000000);
delay(100);
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly: 
  //Dynamixel.readPosition();

  int position;
  Serial.print("tiempo: ");
  Serial.print(millis());
  Serial.print("\n");
  position = posicion(1);
  Serial.print("tiempo2: ");
  Serial.print(millis());
  
  Serial.println(position,DEC);
  //El rango de valores esta entre 68 y 645, hay que poner una bandera que 
  //al leer la posicion inidque si es seguro mover el actuador 
  //delay(1000);
}



int posicion(int ID)
{
int Checksum;
Checksum = (~(ID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;
//     switchCom(Direction_Pin,1);
     digitalWrite (Direction_Pin,1);
    Serial3.write(AX_START);
    Serial3.write(AX_START);
    Serial3.write(ID);
    Serial3.write(AX_POS_LENGTH);
    Serial3.write(AX_READ_DATA);
    Serial3.write(AX_PRESENT_POSITION_L);
    Serial3.write(AX_BYTE_READ_POS);
    Serial3.write(Checksum);
    delayMicroseconds(TX_DELAY_TIME);
    digitalWrite(Direction_Pin,0);
  
    int Position_Long_Byte = -1;
  int Time_Counter = 0;
    while((Serial3.available() < 7) & (Time_Counter < TIME_OUT)){
    Time_Counter++;
    delayMicroseconds(1000);
    }
  
    while (Serial3.available() > 0){
    byte Incoming_Byte = Serial3.read();
    if ( (Incoming_Byte == 255) & (Serial3.peek() == 255) ){
      Serial3.read();                            // Start Bytes
      Serial3.read();                            // Ax-12 ID
      Serial3.read();                            // Length
      byte Error_Byte;
      if( (Error_Byte = Serial3.read()) != 0 )   // Error
        return (Error_Byte*(-1));
    
byte      Position_Low_Byte = Serial3.read();            // Position Bytes
byte      Position_High_Byte = Serial3.read();
      Position_Long_Byte = Position_High_Byte << 8; 
      Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
    }
    }
  return (Position_Long_Byte);     // Returns the read position
}

