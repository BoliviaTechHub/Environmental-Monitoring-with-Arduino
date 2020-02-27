/*   POR GUSTAVO JARRO CALLIZAYA*/
/*DEL*/
/*TEXTO: ENVIRONMENTAL
 *        MONITORING WITH AEDUINO
 *        EMILY GERTZ & PATRICK DI JUSTO
  ) "MAKER PRESS"*/
/* PRACTICA:DETECTING ELECTROMAGNETIC INTERFERENCE
 *          4/PROJECT
  PAGINA 27 DEL TEXTO**/
  

#include <SoftwareSerial.h>
#define SerialIn 9
#define SerialOut 7
#define wDelay 900

int inPin = 5;
int val = 0;

SoftwareSerial mySerialPort(SerialIn, SerialOut);

void setup()
{
  pinMode(SerialOut, OUTPUT);
  pinMode(SerialIn, INPUT);

  mySerialPort.begin(19200);
  mySerialPort.print("vv");

  mySerialPort.print("xxxx");
  delay(wDelay);
  mySerialPort.print("----");
  delay(wDelay);
  mySerialPort.print("8888");
  delay(wDelay);
  mySerialPort.print("xxxx");
  delay(wDelay);

  Serial.begin(9600);

}


void loop()
{

 val = analogRead(inPin);

 Serial.println(val);
 dispData(val);
 val = map(val, 1, 100, 1, 2048);
 tone(9,val,10);

}



void dispData(int i)
{
   if((i<-999) || (i>9999))
 {
   mySerialPort.print("ERRx");
   return;
 }
 char fourChars[5];
 sprintf(fourChars, "%04d", i);

 mySerialPort.print("v");
 mySerialPort.print(fourChars);
  
}
