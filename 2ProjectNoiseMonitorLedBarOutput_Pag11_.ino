/* TRANSCRITO   POR GUSTAVO JARRO CALLIZAYA*/
/*DEL*/
/*TEXTO: ENVIRONMENTAL
 *        MONITORING WITH AEDUINO
 *        EMILY GERTZ & PATRICK DI JUSTO
  ) "MAKER PRESS"*/
/* PRACTICA:NOISE MONITOR LED BAR OUTPUT
 *          2/PROJECT
  PAGINA 11 DEL TEXTO**/

int sensorPin = 2; 

const int numberOfLEDs = 10;

const int numberOfSamples = 16;
int sample;
long signal[numberOfSamples];
long runningAverage;
long sumOfSamples = 0;
int counter =0;
int threshold[] = { 0, 47, 99, 159, 227, 308, 407, 535, 715, 800, 900};

void setup()
{
  pinMode(sensorPin, INPUT);
  
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
  
  /*digitalWrite(3, 0);
  digitalWrite(4, 0);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  digitalWrite(7, 0);
  digitalWrite(8, 0);
  digitalWrite(9, 0);
  digitalWrite(10, 0);
  digitalWrite(11, 0);
  digitalWrite(12, 0);*/
  

  for(int i =0; i <=numberOfSamples; i++)
  {
    sample = digitalRead(sensorPin);

    signal[i] = abs(sample -512);
    sumOfSamples = sumOfSamples + signal[i];
  }

  for(int i=0; i <=numberOfLEDs; i++)
  {
    digitalWrite(i+1, HIGH);
    delay(100);
  }
  
  for(int i=0; i <=numberOfLEDs; i++)
  {
    digitalWrite(i+1, LOW);
    delay(100);
  }


  Serial.begin(9600);

}


void loop()
{
  
  counter = ++counter % numberOfSamples;
  sumOfSamples -= signal[counter];
  sample = digitalRead(sensorPin);
  signal[counter] = abs(sample -512);
  sumOfSamples = sumOfSamples + signal[counter];
  runningAverage = sumOfSamples/numberOfSamples;

  Serial.print("Running Value = ");
  Serial.println(runningAverage);
  Serial.println(" ");
  for (int i =0; i <=numberOfLEDs; i++)
  {
   
    if(runningAverage>threshold[i])
    {
      // if so, light the LED
      digitalWrite(i+1, HIGH);
      delay(10);
    }
  }
  for (int i =numberOfLEDs; i >=1; i--)
  {
    digitalWrite(i+1, LOW);
  }
}
