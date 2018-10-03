//#include <MLX90621.h>



#include <Wire.h>
//#include "I2Cdev.h"

//These are commands
#define CMD_READ_REGISTER 0x02

//Begin registers

#define CAL_ACP 0xD4
#define CAL_BCP 0xD5
#define CAL_alphaCP_L 0xD6
#define CAL_alphaCP_H 0xD7
#define CAL_TGC 0xD8
#define CAL_BI_SCALE 0xD9

#define VTH_L 0xDA
#define VTH_H 0xDB
#define KT1_L 0xDC
#define KT1_H 0xDD
#define KT2_L 0xDE
#define KT2_H 0xDF

//Common sensitivity coefficients
#define CAL_A0_L 0xE0
#define CAL_A0_H 0xE1
#define CAL_A0_SCALE 0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L 0xE4
#define CAL_EMIS_H 0xE5

//KSTA

//Config register = 0xF5-F6

#define OSC_TRIM_VALUE 0xF7

//Bits within configuration register 0x92

#define POR_TEST 10




int refreshRate = 16; //Set this value to your desired refresh frequency

int conta=0;

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int irData[64]; //Contains the raw IR data from the sensor
float temperatures[64]; //Contains the calculated temperatures of each pixel in the array
float Tambient; //Tracks the changing ambient temperature of the sensor
byte eepromData[256]; //Contains the full EEPROM reading from the MLX (Slave 0x50)


//These are constants calculated from the calibration data stored in EEPROM
//See varInitialize and section 7.3 for more information
int v_th, a_cp, b_cp, tgc, b_i_scale;
float k_t1, k_t2, emissivity;
int a_ij[64], b_ij[64];


//These values are calculated using equation 7.3.3.2
//They are constants and can be calculated using the MLX90620_alphaCalculator sketch
float alpha_ij[64] = {
  1.67684E-8, 1.85146E-8, 1.87474E-8, 1.67684E-8, 1.87474E-8, 2.04936E-8, 2.04936E-8, 1.79325E-8, 
  2.00862E-8, 2.20653E-8, 2.16578E-8, 1.93295E-8, 2.10757E-8, 2.32294E-8, 2.28220E-8, 2.04936E-8, 
  2.18324E-8, 2.43936E-8, 2.41607E-8, 2.16578E-8, 2.28220E-8, 2.49756E-8, 2.49756E-8, 2.26473E-8, 
  2.32294E-8, 2.53249E-8, 2.57323E-8, 2.34040E-8, 2.32294E-8, 2.61398E-8, 2.59070E-8, 2.38115E-8, 
  2.32294E-8, 2.59070E-8, 2.61398E-8, 2.39861E-8, 2.29966E-8, 2.57323E-8, 2.61398E-8, 2.38115E-8, 
  2.28220E-8, 2.57323E-8, 2.57323E-8, 2.38115E-8, 2.26473E-8, 2.53249E-8, 2.53249E-8, 2.34040E-8, 
  2.12503E-8, 2.43936E-8, 2.51503E-8, 2.29966E-8, 2.00862E-8, 2.28220E-8, 2.32294E-8, 2.20070E-8, 
  1.81653E-8, 2.08429E-8, 2.22399E-8, 2.04936E-8, 1.61863E-8, 1.95041E-8, 1.99116E-8, 1.85146E-8, 
};

byte loopCount = 0; //Used in main loop
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


void setup()
{
  Serial.begin(115200);  // start serial for output
  Wire.begin();        // join i2c bus (address optional for master)
  delay(6);
  
  read_EEPROM_MLX90620();
  setConfiguration(refreshRate);
  calculate_TA();  
}

void loop()
{

  //readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array
  //Serial.println(" ");
   
  if(loopCount++ == 16) //Tambient changes more slowly than the pixel readings. Update TA only every 16 loops.
  { 
    calculate_TA(); //Calculate the new Tambient

    if(checkConfig_MLX90620()) //Every 16 readings check that the POR flag is not set
    {
      //Serial.println("POR Detected!");
      setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
    }

    loopCount = 0; //Reset count
  }

  readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array
  calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
  conta++;

//Serial.print("tiempo");Serial.print(millis());Serial.print("\n");
  //Tiempo d ejecucion:30 milisegundos
  if(conta>20){
    //Serial.println(millis());
    prettyPrintTemperatures(); //Print the array in a 4 x 16 pattern
    //Serial.println(millis());
    conta=0;
  }


//////////////////////////////////////////////////////////////////////////////////////////////////
  
}





void read_EEPROM_MLX90620()
{
 
/*
  i2c_start_wait(MLX90620_EEPROM_WRITE);
  i2c_write(0x00); //EEPROM info starts at location 0x00
  i2c_rep_start(MLX90620_EEPROM_READ);
  //Read all 256 bytes from the sensor's EEPROM
  for(int i = 0 ; i <= 255 ; i++){
    eepromData[i] = i2c_readAck();

    */

  //I2Cdev::readBytes(0x50,0x00,60,eepromData);
  //I2Cdev::readBytes(0x50,0x05,10,eepromData);
  int8_t count =0;
                Serial.print("eeprom");
                Wire.beginTransmission(0x50);

                Wire.write(0x00);
                
  //              Serial.print("endtransmission");
                Wire.endTransmission();
   //             Serial.print("end" );
                Wire.beginTransmission(0x50);
 //               Serial.print("requestfrom:" );
               for (int i=0;i<=255;i++) 
               {
                Wire.requestFrom(0x50,1);
                if(Wire.available())
                eepromData[i]=Wire.read();
                //count=Wire.read();
                Serial.print(eepromData[i],HEX);
                Serial.print("-");
               }
      varInitialization(eepromData); //Calculate a bunch of constants from the EEPROM data
      writeTrimmingValue(eepromData[OSC_TRIM_VALUE]);
               
  }

//From the 256 bytes of EEPROM data, initialize 
void varInitialization(byte calibration_data[])
{
  v_th = 256 * calibration_data[VTH_H] + calibration_data[VTH_L];
  k_t1 = (256 * calibration_data[KT1_H] + calibration_data[KT1_L]) / 1024.0; //2^10 = 1024
  k_t2 = (256 * calibration_data[KT2_H] + calibration_data[KT2_L]) / 1048576.0; //2^20 = 1,048,576
  emissivity = ((unsigned int)256 * calibration_data[CAL_EMIS_H] + calibration_data[CAL_EMIS_L]) / 32768.0;
  
  a_cp = calibration_data[CAL_ACP];
  if(a_cp > 127) a_cp -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

  b_cp = calibration_data[CAL_BCP];
  if(b_cp > 127) b_cp -= 256;

  tgc = calibration_data[CAL_TGC];
  if(tgc > 127) tgc -= 256;

  b_i_scale = calibration_data[CAL_BI_SCALE];

  for(int i = 0 ; i < 64 ; i++)
  {
    //Read the individual pixel offsets
    a_ij[i] = calibration_data[i]; 
    if(a_ij[i] > 127) a_ij[i] -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

    //Read the individual pixel offset slope coefficients
    b_ij[i] = calibration_data[0x40 + i]; //Bi(i,j) begins 64 bytes into EEPROM at 0x40
    if(b_ij[i] > 127) b_ij[i] -= 256;
  }
  
}


//Given a 8-bit number from EEPROM (Slave address 0x50), write value to MLX sensor (Slave address 0x60)
void writeTrimmingValue(byte val)
{
Wire.beginTransmission(0x60);
    Wire.write(0x04);
    Wire.write((byte)val-0xAA);
    Wire.write(val);
    Wire.write(0x56);
    Wire.write(0x00);
    Wire.endTransmission();  

}
  

//Receives the refresh rate for sensor scanning
//Sets the two byte configuration registers
//This function overwrites what is currently in the configuration registers
//The MLX doesn't seem to mind this (flags are read only)
void setConfiguration(int irRefreshRateHZ)
{
  byte Hz_LSB;

  switch(irRefreshRateHZ)
  {
  case 0:
    Hz_LSB = 0b00001111;
    break;
  case 1:
    Hz_LSB = 0b00001110;
    break;
  case 2:
    Hz_LSB = 0b00001101;
    break;
  case 4:
    Hz_LSB = 0b00001100;
    break;
  case 8:
    Hz_LSB = 0b00001011;
    break;
  case 16:
    Hz_LSB = 0b00001010;
    break;
  case 32:
    Hz_LSB = 0b00001001;
    break;
  default:
    Hz_LSB = 0b00001110;
  }
 byte defaultConfig_H = 0b01110100; // x111.01xx, Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  
    Wire.beginTransmission(0x60);
    Wire.write(0x03);
    Wire.write((byte)Hz_LSB - 0x55);
    Wire.write(Hz_LSB);
    Wire.write(defaultConfig_H - 0x55);
    Wire.write(defaultConfig_H);
    Wire.endTransmission();
 /* i2c_start_wait(MLX90620_WRITE);
  i2c_write(0x03); //Command = configuration value
  i2c_write((byte)Hz_LSB - 0x55);
  i2c_write(Hz_LSB);
  i2c_write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  i2c_write(defaultConfig_H);
  i2c_stop();*/
}


//Gets the latest PTAT (package temperature ambient) reading from the MLX
//Then calculates a new Tambient
//Many of these values (k_t1, v_th, etc) come from varInitialization and EEPROM reading
//This has been tested to match example 7.3.2
void calculate_TA(void)
{
  unsigned int ptat = readPTAT_MLX90620();

  Tambient = (-k_t1 + sqrt((k_t1)*(k_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
}



//Reads the PTAT data from the MLX
//Returns an unsigned int containing the PTAT
unsigned int readPTAT_MLX90620()
{
  byte ptatHigh,ptatLow;
  //Serial.print("pTat ");

   Wire.beginTransmission(0x60);
   //Serial.print("write");
   Wire.write(CMD_READ_REGISTER);
   Wire.write(0x90);
   Wire.write(0x00);
   Wire.write(0x01);
   Wire.endTransmission(0);
   //Serial.print("beginRead:");   
   Wire.beginTransmission(0x60);
  //Serial.print("request");
  Wire.requestFrom(0x60,2);
if(Wire.available())
{    
    //Serial.println("available");
      ptatLow=Wire.read();
    ptatHigh=Wire.read();
}
   //Serial.print("high: ");
   //Serial.print(ptatHigh,DEC);
   //Serial.print("low: ");
   //Serial.println(ptatLow,DEC);
  
  return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}


//Calculate the temperatures seen for each pixel
//Relies on the raw irData array
//Returns an 64-int array called temperatures
void calculate_TO()
{
  float v_ir_off_comp;
  float v_ir_tgc_comp;
  float v_ir_comp;

  //Calculate the offset compensation for the one compensation pixel
  //This is a constant in the TO calculation, so calculate it here.
  int cpix = readCPIX_MLX90620(); //Go get the raw data of the compensation pixel
  float v_cp_off_comp = (float)cpix - (a_cp + (b_cp/pow(2, b_i_scale)) * (Tambient - 25)); 

  for (int i = 0 ; i < 64 ; i++)
  {
    v_ir_off_comp = irData[i] - (a_ij[i] + (float)(b_ij[i]/pow(2, b_i_scale)) * (Tambient - 25)); //#1: Calculate Offset Compensation 

    v_ir_tgc_comp = v_ir_off_comp - ( ((float)tgc/32) * v_cp_off_comp); //#2: Calculate Thermal Gradien Compensation (TGC)

    v_ir_comp = v_ir_tgc_comp / emissivity; //#3: Calculate Emissivity Compensation

    temperatures[i] = sqrt( sqrt( (v_ir_comp/alpha_ij[i]) + pow(Tambient + 273.15, 4) )) - 273.15;
  }
}


//Read the compensation pixel 16 bit data
int readCPIX_MLX90620()
{

    byte cpixLow; //Grab the two bytes
    byte cpixHigh;

Wire.beginTransmission(0x60);
   Wire.write(CMD_READ_REGISTER);
   //Wire.beginTransmission(0x60);
   Wire.write(0x91);
   Wire.write(0x00);
   Wire.write(0x01);
   //Serial.print("endTransmission ");
   Wire.endTransmission(0);
   //Serial.print("beginRead:");   
   Wire.beginTransmission(0x60);
        
   Wire.requestFrom(0x60,2);
   if(Wire.available())
    {
       cpixLow=Wire.read();
       cpixHigh=Wire.read();                  
    } 

  /*
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read register
  i2c_write(0x91);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(MLX90620_READ);

  byte cpixLow = i2c_readAck(); //Grab the two bytes
  byte cpixHigh = i2c_readAck();
  i2c_stop();
*/
  return ( (int)(cpixHigh << 8) | cpixLow);
}





//Reads 64 bytes of pixel data from the MLX
//Loads the data into the irData array
void readIR_MLX90620()
{
  
  byte pixelDataLow,pixelDataHigh;
    //Serial.print("readIR ");
   Wire.beginTransmission(0x60);
   Wire.write(CMD_READ_REGISTER);
   //Wire.beginTransmission(0x60);
   Wire.write(0x00);
   Wire.write(0x01);
   Wire.write(0x40);
   //Serial.print("endTransmission ");
   Wire.endTransmission(0);
   //Serial.print("beginRead:");   
   Wire.beginTransmission(0x60);
                  pixelDataLow=Wire.requestInit(0x60);
                  pixelDataHigh=Wire.requestCont(0x60);
                  irData[0] = (int)(pixelDataHigh << 8) | pixelDataLow;                  
   //               Serial.print(irData[0],DEC);
    //              Serial.print("-");
     for (int i=1;i <= 62;i++) 
               {
                  pixelDataLow=Wire.requestCont(0x60);
                  pixelDataHigh=Wire.requestCont(0x60);
                  
                  irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;                  
                                        
      //          Serial.print(irData[i],DEC);
        //        Serial.print("-");
               }
                  pixelDataLow=Wire.requestCont(0x60);
                  pixelDataHigh=Wire.requestEnd(0x60);
                  irData[63] = (int)(pixelDataHigh << 8) | pixelDataLow;                  
          //        Serial.print(irData[63],DEC);
          //        Serial.print("-");
   
}

//Reads the current configuration register (2 bytes) from the MLX
//Returns two bytes
unsigned int readConfig_MLX90620()
{
  byte configLow; //Grab the two bytes
  byte configHigh;

Wire.beginTransmission(0x60);
   Wire.write(CMD_READ_REGISTER);
   //Wire.beginTransmission(0x60);
   Wire.write(0x92);
   Wire.write(0x00);
   Wire.write(0x01);
   //Serial.print("readConfig ");
   Wire.endTransmission(0);
   //Serial.print("beginRead:");   
   Wire.beginTransmission(0x60);
        
   Wire.requestFrom(0x60,2);
   if(Wire.available())
    {
       configLow=Wire.read();
       configHigh=Wire.read();                  
    }              
                
              
  /*
  i2c_start_wait(MLX90620_WRITE); //The MLX configuration is in the MLX, not EEPROM
  i2c_write(CMD_READ_REGISTER); //Command = read configuration register
  i2c_write(0x92); //Start address
  i2c_write(0x00); //Address step of zero
  i2c_write(0x01); //Number of reads is 1

    i2c_rep_start(MLX90620_READ);

  byte configLow = i2c_readAck(); //Grab the two bytes
  byte configHigh = i2c_readAck();

  i2c_stop();
*/
  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
}

//Poll the MLX for its current status
//Returns true if the POR/Brown out bit is set
boolean checkConfig_MLX90620()
{
  if ( (readConfig_MLX90620() & (unsigned int)1<<POR_TEST) == 0)
    return true;
  else
    return false;
}

//Prints the temperatures in a way that's more easily viewable in the terminal window
void prettyPrintTemperatures()
{
  Serial.println();
  for(int i = 0 ; i < 64 ; i++)
  {
    if(i % 16 == 0) Serial.println();
    Serial.print(temperatures[i]);
    //Serial.print(irData[i]);
    Serial.print(", ");
  }
}

//Prints the temperatures in a way that's more easily parsed by a Processing app
//Each line starts with '$' and ends with '*'
void rawPrintTemperatures()
{
  Serial.print("$");
  for(int i = 0 ; i < 64 ; i++)
  {
    Serial.print(temperatures[i]);
    if(i!=63){
      Serial.print(","); //Don't print comma on last temperature
    }
  }
  Serial.println("*");
}

//Given a Celsius float, converts to Fahrenheit
float convertToFahrenheit (float Tc)
{
  float Tf = (9/5) * Tc + 32;

  return(Tf);
}


