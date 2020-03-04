/* TRANSCRITO   POR GUSTAVO JARRO CALLIZAYA*/
/*DEL*/
/*TEXTO: ENVIRONMENTAL
 *        MONITORING WITH AEDUINO
 *        EMILY GERTZ & PATRICK DI JUSTO
  ) "MAKER PRESS"*/
/* PRACTICA: HYMIDITY, TEMPERATURE Y DEW POINT/ 4 CHAR DISPLAY
 *          7/PROJECT
  PAGINA 53 DEL TEXTO**/
  
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <stdlib.h>
#define DHT22_ERROR_VALUE -99.5
#define DHT22_PIN 4

typedef enum
{
  DHT_ERROR_NONE = 0,
  DHT_BUS_HUNG,
  DHT_ERROR_NOT_PRESENT,
  DHT_ERROR_ACK_TOO_LONG,
  DHT_ERROR_SYNC_TIMEOUT,
  DHT_ERROR_DATA_TIMEOUT,
  DHT_ERROR_CHECKSUM,
  DHT_ERROR_TOOQUICK
} DHT22_ERROR_t;

class DHT22
{
  private:
    uint8_t _bitmask;
    volatile uint8_t *_baseReg;
    unsigned long _lastReadTime;
    float _lastHumidity;
    float _lastTemperature;

  public:
    DHT22(uint8_t pin);
    DHT22_ERROR_t readData(void);
    float getHumidity();
    float getTemperatureC();
    void clockReset();
};


// Setup a DHT22 instance
DHT22 myDHT22(DHT22_PIN);


#define SerialIn 2
#define SerialOut 3


#define WDelay 900


uint8_t thou=0;
uint8_t hund=0;
uint8_t tens=0;
uint8_t ones=0;


SoftwareSerial mySerialPort(SerialIn, SerialOut);


void setup(void)
{


// start serial port
Serial.begin(9600);
Serial.println("DHT22 Library Demo - Temperatura y Humedad");
Serial.println("Temperatura");
Serial.println("Humedad");
pinMode(SerialOut, OUTPUT);
pinMode(SerialIn, INPUT);


mySerialPort.begin(9600);
mySerialPort.print("v");

mySerialPort.print("xxxx");
delay(WDelay);
mySerialPort.print("----");
delay(WDelay);
mySerialPort.print("8888");
delay(WDelay);
mySerialPort.print("xxxx");
delay(WDelay);


}






void loop(void)
{
float tempC;
float tempF;
float humid;
float dewPoint;




DHT22_ERROR_t errorCode;


delay(2000);


errorCode = myDHT22.readData();

Serial.print(errorCode);

switch(errorCode)

{
 
        case DHT_ERROR_NONE:
        Serial.print("Temperature: ");
        tempC = myDHT22.getTemperatureC();
                 
        Serial.print(tempC);
        Serial.print("C Humidity: ");
        
        dispData((int)tempC, 'C');
        
        tempF = (tempC*1.8)+32;
        
        
        
        delay(WDelay);
        dispData((int) tempF, 'F');
        delay(WDelay);
        
        humid = myDHT22.getHumidity();
        
        Serial.print(humid);
        Serial.println("%");
        
        dispData((int)humid, 'h');
        delay(WDelay);
        
        dewPoint = calculateDewpoint(tempC, humid);
        dispData((int) dewPoint, 'd');
 
        Serial.print(dewPoint);
        Serial.println("d");
        
        delay(WDelay);


        
        break;
        case DHT_ERROR_CHECKSUM:
        Serial.print("Error Cheksum");
        break;
        case DHT_BUS_HUNG:
        Serial.print("Bus Hung");
        break;
        case DHT_ERROR_NOT_PRESENT:
        Serial.print("Not Present");
        break;
        case DHT_ERROR_ACK_TOO_LONG:
        break;
        case DHT_ERROR_SYNC_TIMEOUT:
        break;
        case DHT_ERROR_DATA_TIMEOUT:
        break;
        case DHT_ERROR_TOOQUICK:
        break;
}




}




float calculateDewpoint(float T, float RH)
{
// approximate dewpoint using the formula from wikipedia's article on dewpoint


float dp = 0.0;
float gTRH = 0.0;
float a = 17.271;
float b = 237.7;


gTRH = ((a*T)/(b+T))+log(RH/100);
dp = (b*gTRH)/(a-gTRH);


return dp;
}



void dispData(int i, char c)
{

 if(c == 'k' || c=='K' || c=='m' || c=='l' || c == 'v' || c=='V' || c=='W' || c=='Z' || c=='w' || c=='z')
 {
        mySerialPort.print("bAdx");
        return;
        
 }
 
   
 if((i<-999) || (i>9999))
 {
   mySerialPort.print("ERRx");
   return;
 }

 mySerialPort.print("v");

 if (i > 999) { // i between 1000 and 9999 inclusive

   mySerialPort.print(i, DEC);

 } else if (i > 99) { // i between 100 and 999, inclusive

   mySerialPort.print(i, DEC);
   mySerialPort.print(c);

 } else if (i > 9) { // i between 10 and 99 inclusive

   mySerialPort.print(i, DEC);
   mySerialPort.print("x");
   mySerialPort.print(c);

 } else if (i > 0) { // i between 1 and 9 inclusive

   mySerialPort.print("x");
   mySerialPort.print(i, DEC);
   mySerialPort.print("x");
      mySerialPort.print(c);

 } else if (i < -99) { // i between -100 and -999, inclusive

   mySerialPort.print(i, DEC);
   mySerialPort.print(c);

 } else if (i < -9) { // i between -10 and -99, inclusive

   mySerialPort.print(i, DEC);
   mySerialPort.print(c);

 } else if (i < 0) { // i between -1 and -9 inclusive

   mySerialPort.print(i, DEC);
   mySerialPort.print("x");
   mySerialPort.print(c);

 }

}








 
#include "Arduino.h"

extern "C" {
//#include "WConstants.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
}

#define DIRECT_READ(base, mask)    (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask) ((*(base+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)  ((*(base+2)) &= ~(mask))
//#define DIRECT_WRITE_HIGH(base, mask) ((*(base+2)) |= (mask))

// This should be 40, but the sensor is adding an extra bit at the start
#define DHT22_DATA_BIT_COUNT 41

DHT22::DHT22(uint8_t pin)
{
    _bitmask =  digitalPinToBitMask(pin);
    _baseReg = portInputRegister(digitalPinToPort(pin));
    _lastReadTime = millis();
    _lastHumidity = DHT22_ERROR_VALUE;
    _lastTemperature = DHT22_ERROR_VALUE;
}

//
// Read the 40 bit data stream from the DHT 22
// Store the results in private member data to be read by public member functions
//
DHT22_ERROR_t DHT22::readData()
{
  uint8_t bitmask = _bitmask;
  volatile uint8_t *reg asm("r30") = _baseReg;
  uint8_t retryCount;
  uint8_t bitTimes[DHT22_DATA_BIT_COUNT];
  int currentHumidity;
  int currentTemperature;
  uint8_t checkSum, csPart1, csPart2, csPart3, csPart4;
  unsigned long currentTime;
  int i;

  currentHumidity = 0;
  currentTemperature = 0;
  checkSum = 0;
  currentTime = millis();
  for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
  {
    bitTimes[i] = 0;
  }

  if(currentTime - _lastReadTime < 2000)
  {
    // Caller needs to wait 2 seconds between each call to readData
    return DHT_ERROR_TOOQUICK;
  }
  _lastReadTime = currentTime;

  // Pin needs to start HIGH, wait until it is HIGH with a timeout
  cli();
  DIRECT_MODE_INPUT(reg, bitmask);
  sei();
  retryCount = 0;
  do
  {
    if (retryCount > 125)
    {
      return DHT_BUS_HUNG;
    }
    retryCount++;
    delayMicroseconds(2);
  } while(!DIRECT_READ(reg, bitmask));
  // Send the activate pulse
  cli();
  DIRECT_WRITE_LOW(reg, bitmask);
  DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
  sei();
  delayMicroseconds(1100); // 1.1 ms
  cli();
  DIRECT_MODE_INPUT(reg, bitmask);  // Switch back to input so pin can float
  sei();
  // Find the start of the ACK Pulse
  retryCount = 0;
  do
  {
    if (retryCount > 25) //(Spec is 20 to 40 us, 25*2 == 50 us)
    {
      return DHT_ERROR_NOT_PRESENT;
    }
    retryCount++;
    delayMicroseconds(2);
  } while(!DIRECT_READ(reg, bitmask));
  // Find the end of the ACK Pulse
  retryCount = 0;
  do
  {
    if (retryCount > 50) //(Spec is 80 us, 50*2 == 100 us)
    {
      return DHT_ERROR_ACK_TOO_LONG;
    }
    retryCount++;
    delayMicroseconds(2);
  } while(DIRECT_READ(reg, bitmask));
  // Read the 40 bit data stream
  for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
  {
    // Find the start of the sync pulse
    retryCount = 0;
    do
    {
      if (retryCount > 35) //(Spec is 50 us, 35*2 == 70 us)
      {
        return DHT_ERROR_SYNC_TIMEOUT;
      }
      retryCount++;
      delayMicroseconds(2);
    } while(!DIRECT_READ(reg, bitmask));
    // Measure the width of the data pulse
    retryCount = 0;
    do
    {
      if (retryCount > 50) //(Spec is 80 us, 50*2 == 100 us)
      {
        return DHT_ERROR_DATA_TIMEOUT;
      }
      retryCount++;
      delayMicroseconds(2);
    } while(DIRECT_READ(reg, bitmask));
    bitTimes[i] = retryCount;
  }
  // Now bitTimes have the number of retries (us *2)
  // that were needed to find the end of each data bit
  // Spec: 0 is 26 to 28 us
  // Spec: 1 is 70 us
  // bitTimes[x] <= 11 is a 0
  // bitTimes[x] >  11 is a 1
  // Note: the bits are offset by one from the data sheet, not sure why
  for(i = 0; i < 16; i++)
  {
    if(bitTimes[i + 1] > 11)
    {
      currentHumidity |= (1 << (15 - i));
    }
  }
  for(i = 0; i < 16; i++)
  {
    if(bitTimes[i + 17] > 11)
    {
      currentTemperature |= (1 << (15 - i));
    }
  }
  for(i = 0; i < 8; i++)
  {
    if(bitTimes[i + 33] > 11)
    {
      checkSum |= (1 << (7 - i));
    }
  }

  _lastHumidity = (float(currentHumidity & 0x7FFF) / 10.0);
  if(currentTemperature & 0x8000)
  {
   // Below zero, non standard way of encoding negative numbers!
    currentTemperature &= 0x7FFF;
    _lastTemperature = (float(currentTemperature) / 10.0) * -1.0;
  }
  else
  {
    _lastTemperature = float(currentTemperature) / 10.0;
  }

  csPart1 = currentHumidity >> 8;
  csPart2 = currentHumidity & 0xFF;
  csPart3 = currentTemperature >> 8;
  csPart4 = currentTemperature & 0xFF;
  if(checkSum == ((csPart1 + csPart2 + csPart3 + csPart4) & 0xFF))
  {
    return DHT_ERROR_NONE;
  }
  return DHT_ERROR_CHECKSUM;
}

float DHT22::getHumidity()
{
  return _lastHumidity;
}

float DHT22::getTemperatureC()
{
  return _lastTemperature;
}

//
// This is used when the millis clock rolls over to zero
//
void DHT22::clockReset()
{
  _lastReadTime = millis();

}
