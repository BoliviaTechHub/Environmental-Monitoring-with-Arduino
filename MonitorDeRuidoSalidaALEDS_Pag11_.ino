/*   POR GUSTAVO JARRO CALLIZAYA*/
/*DEL*/
/*TEXTO: ENVIRONMENTAL
 *        MONITORING WITH AEDUINO
 *        EMILY GERTZ & PATRICK DI JUSTO
  ) "MAKER PRESS"*/
/* PRACTICA:NOISE MONITOR LED BAR OUTPUT
 *          2/PROJECT
  PAGINA 11 DEL TEXTO**/
  /*SENSOR KY - 37*/
int soundsensor = 2;


void setup() {
  // put your setup code here, to run once:
pinMode(soundsensor, INPUT);

  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
int sensorvalue = digitalRead(soundsensor);
if(sensorvalue == 1)
{
  /*digitalWrite(led1, 1);
  digitalWrite(led3, 1);
  delay(10);
  digitalWrite(led2, 1);
  digitalWrite(led4, 1);
  delay(10);*/
  digitalWrite(3, 1);
  digitalWrite(4, 1);
  digitalWrite(5, 1);
  digitalWrite(6, 1);
  digitalWrite(7, 1);
  digitalWrite(8, 1);
  digitalWrite(9, 1);
  digitalWrite(10, 1);
  digitalWrite(11, 1);
  digitalWrite(12, 1);
  delay(10);
}
else
{
   digitalWrite(3, 0);
  digitalWrite(4, 0);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  digitalWrite(7, 0);
  digitalWrite(8, 0);
  digitalWrite(9, 0);
  digitalWrite(10, 0);
  digitalWrite(11, 0);
  digitalWrite(12, 0);
  
}
}
