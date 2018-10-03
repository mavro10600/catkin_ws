

//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>
#include <SPI.h>

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
//Messenger object
Messenger Messenger_Handler = Messenger();

///////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];


#define OUTPUT_READABLE_QUATERNION
////////////////////////////////////////////////////////////////////////////////////////////////
//#define OUTPUT_READABLE_YAWPITCHROLL
//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}





//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset

#define RESET_PIN PB_2

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

////////////////////////////////////////////////////////////////////////////////////////////
//AS5043 variables
const int chipSelectPin2 = PB_0;//pin ss encodder1
const int chipSelectPin1 = PB_5; //pin de seleccion de esclavo 2


//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup()
{

  //Init Serial port with 115200 baud rate
  Serial.begin(115200);
  //Serial.begin(230400);
  Setup_MPU6050();
  //Setup Reset pins
  SetupReset();
  //Set up Messenger
  Messenger_Handler.attach(OnMssageCompleted);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 function

void Setup_MPU6050()
{


  Wire.begin();
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // initialize device
  Serial.println("Initializing I2C devices...");
  //accelgyro.initialize();
  Serial.println("empieza");
  if (I2Cdev::writeBits(0x68, 0x6B, 2, 3, 0x01))
  {
    Serial.println("1");
  }
  else {
    Serial.println("0");
  }
  if (I2Cdev::writeBits(0x68, 0x1B, 4, 2, 0x00))
  {
    Serial.println("10");
  }
  else {
    Serial.println("00");
  }
  if (I2Cdev::writeBits(0x68, 0x1C, 4, 2, 0x00))
  {
    Serial.println("100");
  }
  else {
    Serial.println("000");
  }
  if (I2Cdev::writeBit(0x68, 0x6B, 6, false))
  {
    Serial.println("1000");
  }
  else {
    Serial.println("0000");
  }
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Initialize DMP in MPU 6050
  Setup_MPU6050_DMP();


}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU 6050 DMP
void Setup_MPU6050_DMP()
{

  //DMP Initialization

  devStatus = accelgyro.dmpInitialize();

  accelgyro.setXGyroOffset(220);
  accelgyro.setXGyroOffset(76);
  accelgyro.setXGyroOffset(-85);
  accelgyro.setXGyroOffset(1788);

  Serial.println(devStatus);
  if (devStatus == 0) {

    accelgyro.setDMPEnabled(true);
    Serial.println("2");
    pinMode(PUSH2, INPUT_PULLUP);
    attachInterrupt(PUSH2, dmpDataReady, RISING);

    mpuIntStatus = accelgyro.getIntStatus();

    dmpReady = true;

    packetSize = accelgyro.dmpGetFIFOPacketSize();
    Serial.println("3");
  } else {

    ;
  }


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Reset() function

void SetupReset()

{


  pinMode(GREEN_LED, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);


  ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
  digitalWrite(RESET_PIN, HIGH);


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

void loop()
{

  //Read from Serial port
  Read_From_Serial();


  //Send time information through serial port
  //Update_Time();


  //Send MPU 6050 values through serial port
  Update_MPU6050();



}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
  while (Serial.available() > 0)
  {

    int data = Serial.read();
    Messenger_Handler.process(data);


  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{

  char reset[] = "r";
  char set_speed[] = "s";

  if (Messenger_Handler.checkString(reset))
  {

    Serial.println("Reset Done");
    Reset();

  }
  if (Messenger_Handler.checkString(set_speed))
  {

    //This will set the speed
    //Set_Speed();
    return;


  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reset function
void Reset()
{

  digitalWrite(GREEN_LED, HIGH);
  delay(1000);
  digitalWrite(RESET_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050

void Update_MPU6050()
{



  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  ///Update values from DMP for getting rotation vector
  Update_MPU6050_DMP();
  //Serial.println("200");

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050 DMP functions

void Update_MPU6050_DMP()
{

  //DMP Processing
  //Serial.println("not");
  if (!dmpReady) {
    return;
  }

  /*
      while (!mpuInterrupt && fifoCount < packetSize) {

         ;

            }
  */

  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();

  //get current FIFO count
  fifoCount = accelgyro.getFIFOCount();


  if ((mpuIntStatus & 0x10) || fifoCount > 512 ) {
    // reset so we can continue cleanly
    accelgyro.resetFIFO();
  }




  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);


    Serial.print("i"); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
    Serial.print(q.w);
    Serial.print("\n");


#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif


  }


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



