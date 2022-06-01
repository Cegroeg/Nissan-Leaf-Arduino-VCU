// Arduino Mega
// Uses MCP2515 CAN 
#include <SPI.h>
#include "mcp_can.h"


long unsigned int rxId;

unsigned long rcvTime;
unsigned long LastTxTime;
unsigned long LastTxTime2;

unsigned char len = 0;
unsigned char buf[8];
unsigned char CanBuff[8];
unsigned char Counter = 0;
//unsigned char NissanCrc = 0;
unsigned char Msg11A[8] = {0x4e,0x40,0x00,0xaa,0xc0,0x00,0,0};
unsigned char Msg1D4[8] = {0x6e,0x6e,0x00,0x00,0x00,0x44,0x01,0};
unsigned char Msg50B[6] = {0x00,0x00,0x06,0xc0,0x00,0x60};
unsigned char DTCreq[8] = {0x01,0x03,0x00,0x00,0x00,0x00,0x00,0};
unsigned char Test[8] ={0xF7, 0x15, 0x06, 0x10, 0x47, 0x44, 0x00, 0};//D9 

const int MCP_CS_PIN = 7;//9;
const int Throttle = A1;

int Torque = 0;
//int LastTorque = 0;
unsigned int Rpm = 0;
unsigned int Revs = 0;
unsigned int LastRpm = 0;
const int RpmLimit = 500;
const int MaxTorque = 200;
unsigned int Trq = 0;
byte sndStat;
bool Reverse = 0;

MCP_CAN CAN(MCP_CS_PIN); 

#define CS1  26 // SpiADC chip select
#define ADE7913_WRITE   0
#define ADE7913_READ    4
#define ADE7913_AMP_READING  0
#define ADE7913_ADC1_READING  1 << 3
#define ADE7913_ADC2_READING  2 << 3
#define ADE7913_CONFIG  8 << 3
#define ADE7913_STATUS0 9 << 3

#define ADCoffset  189
#define ADCgain  1024

#define Batt 2
#define PreCharge 3
#define Out3 4
#define Out4 5 
#define PreChargeTime 3000
#define Brake 48
#define ReverseGear 49

#define TorqueStep 10

SPISettings ADC_spi_settings(1000000, MSBFIRST, SPI_MODE3);   

//***********************************************************************************
void InitSpiADC()
{
  byte ADCresult;
  do
  {
        SPI.beginTransaction(ADC_spi_settings);
        SPI.transfer(0);
        SPI.endTransaction();
        Serial.println("Wait for SpiADC ready");
        SPI.beginTransaction(ADC_spi_settings);
        digitalWrite(CS1, LOW); //select first ADC chip
        SPI.transfer(ADE7913_READ | ADE7913_STATUS0);
        ADCresult = SPI.transfer(0);
        digitalWrite(CS1, HIGH);
        SPI.endTransaction();
   }
   while( (ADCresult&1) == true);  
        
   //sysioState = SYSSTATE_ADC1OK;
   Serial.println("SpiADC is ready. Set clock out enable and ADC_FREQ to 2khz");
   //Now enable the CLKOUT function on first unit so that the other two will wake up
   SPI.beginTransaction(ADC_spi_settings);
   digitalWrite(CS1, LOW);
   SPI.transfer(ADE7913_WRITE |  ADE7913_CONFIG);
   SPI.transfer(1 | 2 << 4); //Set clock out enable and ADC_FREQ to 2khz
   digitalWrite(CS1, HIGH);
   SPI.endTransaction();
 
}

//***********************************************************************************
int32_t ReadSpiADC()
{
    int32_t result;
    int32_t byt;
    
    SPI.beginTransaction(ADC_spi_settings);
    digitalWrite(CS1, LOW);
    //SPI.transfer(ADE7913_READ | ADE7913_AMP_READING);
    SPI.transfer(ADE7913_READ | ADE7913_ADC1_READING);
    //SPI.transfer(ADE7913_READ | ADE7913_ADC2_READING);
    byt = SPI.transfer(0);
    result = (byt << 16);
    byt = SPI.transfer(0);
    result = result + (byt << 8);
    byt = SPI.transfer(0);
    digitalWrite(CS1, HIGH);
    SPI.endTransaction();
    
    result = result + byt;
    //now we've got the whole 24 bit value but it is a signed 24 bit value so we must sign extend
    if (result & (1 << 23)) result |= (255 << 24);

    // Convert 24 bit value to 16 bits and apply offset & gain...
    result = result / 2048;
    result -= ADCoffset;
    result = (result * ADCgain) / 1024;
    
    return result;
}

//***********************************************************************************
static void Nissan_crc(uint8_t *data) 
{
    data[7] = 0;
    uint8_t crc = 0;
    for (int b=0; b<8; b++) 
    {
      for (int i=7; i>=0; i--) 
      {
        uint8_t bit = ((data[b] &(1 << i)) > 0) ? 1 : 0;
        if(crc >= 0x80) crc = (byte)(((crc << 1) + bit) ^ 0x85);
        else            crc = (byte)((crc << 1) + bit);
      }
    }
    data[7] = crc;
}
  
//***********************************************************************************  
void setup()
{
    pinMode(Batt,OUTPUT);
    pinMode(PreCharge,OUTPUT);
    pinMode(Out3,OUTPUT);
    pinMode(Out4,OUTPUT);
    pinMode(Brake,INPUT);
    pinMode(ReverseGear,INPUT);
    
    digitalWrite(Batt,LOW);
    digitalWrite(PreCharge,LOW);
    digitalWrite(Out3,LOW);
    digitalWrite(Out4,LOW);
  
    Serial.begin(115200);
    Serial.println("Leaf VCU V3.1");

    pinMode(CS1, OUTPUT); //Chip Select for first ADC chip
    digitalWrite(CS1,HIGH);
    
    SPI.begin();
    InitSpiADC();
  
    while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_8MHz)) // init can bus : baudrate = 500k (8MHz crystal instead of 16MHz)
    {
        Serial.println("CAN BUS Module Failed to Initialize");
        Serial.println("Retrying....");
        delay(200);
    }    
    Serial.println("CAN BUS Module Initialized!");
    //Serial.println("Time\t\tPGN\tByte0\tByte1\tByte2\tByte3\tByte4\tByte5\tByte6\tByte7"); 

    CAN.setMode(MODE_NORMAL);   // Set operation mode to normal so the MCP2515 sends acks to received data.

    Serial.println("PreCharge...");
    digitalWrite(Batt,HIGH);
    delay(PreChargeTime);
    digitalWrite(PreCharge,HIGH);
    Serial.print("PreCharge done - ");
    Serial.print(PreChargeTime /1000);
    Serial.println("s.");
}

////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    if(CAN_MSGAVAIL == CAN.checkReceive())  
    {
        rcvTime = millis();
        
        //Serial.println("Can Rx...");
        
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        rxId = CAN.getCanId();
        
        //Serial.print("Can ID: ");
        //Serial.println(rxId,HEX);

        if(rxId == 0x1DA )
        {   
          LastRpm = Rpm;       
          Rpm = ( (buf[4]<<8)+ buf[5])/2;
          Revs = Rpm;
          if(Rpm > RpmLimit) Rpm = LastRpm;
          //Serial.print( Rpm ); 
          //Serial.print(" RPM"); 
          //Serial.println();
         
          if(Counter++ > 3) Counter = 0;
          memcpy( CanBuff,Msg11A,8 );
          CanBuff[6] = Counter;
          Nissan_crc(CanBuff);
          sndStat = CAN.sendMsgBuf(0x11A,0, 8, CanBuff);
          if(sndStat != CAN_OK) Serial.println("Error Sending Message...");

          // Send Message 0x1d4...
          unsigned int T = 0;
          for( int i = 0;i<10;i++)
          {
            delay(1);
            T = T + ReadSpiADC();
            //T = T + analogRead(Throttle);
          }
          Trq = T/10;

          //Serial.println(Trq);
          //Serial.println(Rpm);

          //LastTorque = Torque;
          Torque = constrain(Trq,0, RpmLimit);
          Torque = map(Trq,0,RpmLimit,0,MaxTorque);

          if(digitalRead(Brake) == 0) Torque = 0;  //if brake is pressed, zero the throttle value.
          if(Torque == 0)
          {
            if(digitalRead(ReverseGear)) Reverse = true;  //if reverse
            else
            Reverse = false;
          }
          //Reverse = true; /////// Test //////////////////////////////////////////////
          if( Reverse ) Torque = (~Torque) +1; // 2s compliment to make it negative

          memcpy( CanBuff,Msg1D4,8 );
          if(Torque >= -1120 && Torque < 1120) // maximum torque for standard Leaf motor
          {
            Torque = Torque << 4; // 12 bit value
            CanBuff[2] = highByte(Torque); 
            if( Torque < 0 ) bitSet(CanBuff[2],7); // make sure torque is negative
            CanBuff[3] = (lowByte(Torque) & 0xF0 ); // make sure low nibble is 0000
          } 
          else
          {
            CanBuff[2] = 0x00;
            CanBuff[3] = 0x00;
          } 
          CanBuff[4] = 0x7|(Counter<<6);  
          Nissan_crc(CanBuff);
          sndStat = CAN.sendMsgBuf(0x1D4,0, 8, CanBuff);
          if(sndStat != CAN_OK) Serial.println("Error Sending Message...");
        }
        
        if(rxId == 0x55A )
        {
          sndStat = CAN.sendMsgBuf(0x50B,0, 6, Msg50B);
          if(sndStat != CAN_OK) Serial.println("Error Sending Message...");
        }
    }   
}
