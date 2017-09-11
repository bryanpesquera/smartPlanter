/* AUTHOR: Bryan Pesquera DATE: Aug. 22, 2017. */

#include<math.h>

/* MODULES */
// DHT TEMPERATURE/HUMIDITY SENSOR-----
#include<dht.h>
#include<OneWire.h>
#include<DallasTemperature.h>
dht DHT11;
int dhtdata = 5; //dht11
#define owdata 2 //onewire
OneWire ONEWIRE(owdata);
DallasTemperature DS18B20(&ONEWIRE);
// DHT TEMPERATURE/HUMIDITY SENSOR------

// SOIL MOISTURE SENSOR-----------------
int analogPin = A0; 
int analogVal;
int digitalPin = 9;
int digitalVal; //1 is dry, 0 is wet
// SOIL MOISTURE SENSOR-----------------

// DIGIT LIGHT SENSOR-------------------
#define ANALOG_SENSOR_PIN  A1 
#define DIGITAL_SENSOR_PIN 3
int  switch_state;  /* Holds the last digital value */
int  LightAnalogValue; /* Holds the last analog value */
// DIGIT LIGHT SENSOR-------------------

// BLUETOOTH MODULE--------------------
#include <SoftwareSerial.h>// import the serial library
SoftwareSerial Genotronex(11, 10); // RX, TX
int BluetoothData; // the data given from Computer
// BLUETOOTH MODULE--------------------

// SPEAKER-----------------------------
int speakerPin = 6; 
// SPEAKER-----------------------------

// LED---------------------------------
int ledB = 3;
int ledR = 2;
int ledG = 4;
// LED---------------------------------
/* END OF MODULES */

void setup() 
{
  //BEGIN GENOTRONEX AND SERIAL
  Genotronex.begin(9600);
  Serial.begin(9600);

  //LED AND SPEAKER PINMODE
  pinMode(speakerPin, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);

  //CHECK FOR BT CONNECTION

  
  delay(10000);
  
  //SET DHTDATA MODULE
  pinMode(dhtdata, INPUT); //set data type for dht11 sensor
  DS18B20.begin();

  delay(2000);
}

void loop() 
{ 
  //get values from sensors to variables
  int TC = dhtTSensor(); //temp in C
  int TF = TC*1.8+32; //convert to F
  int H = dhtHSensor();
  int L = lightSensor();
  int M = moistureSensorA();
  int HI = getHeatIndex(TF, H);
  
  //set front led
  setLEDHI(HI);

  //send info to chip via BT
  if(bluetoothInteract()==true)
  {
    //print in special chip-arduino format
    printStats2(TC, TF, H, L, M);
  }

  //refresh values every 10 seconds
  delay(10000);
}

//easy to read stats (for human reading)
void printStats(int TC, int TF, int H, int L, int M)
{
  Serial.print("Temperature: " + String(TC) + "C");
  Serial.println(" | " + String(TF) + "F");
  Serial.println("Humidity: " + String(H) + "%");
  Serial.println("Moisture: " + String(M) + "%");
  Serial.println("Light: " + String(L));
  Serial.println("HI: " + String(getHeatIndex(TF, H)));
  Genotronex.print("Temperature: " + String(TC) + "C");
  Genotronex.println(" | " + String(TF) + "F");
  Genotronex.println("Humidity: " + String(H) + "%");
  Genotronex.println("Moisture: " + String(M) + "%");
  Genotronex.println("Light: " + String(L));
  Genotronex.println("HI: " + String(getHeatIndex(TF, H)));
}

//easy to manage stats (communicating with chip)
void printStats2(int TC, int TF, int H, int L, int M)
{
  Genotronex.print(TC);
  Genotronex.println(H);
  Genotronex.println(M);
  Genotronex.println(L);
  Genotronex.println(getHeatIndex(TF, H));
}

int moistureSensorA()
{
  analogVal= analogRead(analogPin);
  analogVal = map(analogVal,550,0,0,100);
  return analogVal;
 }

 bool moistureSensorD(){
  analogVal= analogRead(analogPin);
  analogVal = map(analogVal,550,0,0,100);
  //1 for dry, 0 for wet
  digitalVal = digitalRead(digitalPin);
  return digitalVal;
 }

int dhtTSensor()
 {
  int chk = DHT11.read11(dhtdata);
  return DHT11.temperature;
 }

 int dhtHSensor()
 {
    int chk = DHT11.read11(dhtdata);
    return DHT11.humidity;
 }

int lightSensor()
 {
  switch_state = digitalRead(DIGITAL_SENSOR_PIN);  
  //if (switch_state == LOW) Serial.println("Digital Signal ON ");
  LightAnalogValue = analogRead(ANALOG_SENSOR_PIN);  //Read the voltage from sensor
  LightAnalogValue = (LightAnalogValue, DEC);
  return LightAnalogValue;
 }

bool bluetoothInteract()
 {
 bool rval;
  //tell chip, arduino is ready 
  if (Genotronex.available()){
    Genotronex.println("Arduino ready!");
    delay(500);
    //wait to see if chip is ready
    if(Genotronex.read() == "CHIP ready!"){
      Genotronex.println("Ok!");
      rval = true;
    }
    else rval = false;
 }
 return rval;
}

int getHeatIndex(float T, float R)
 {
  //calculate heat index
  int HI = (0.363445176) + (0.988622465)*T + (4.777114035)*R + (-0.114037667)*T*R + (-8.50208*pow(10, -4))*pow(T, 2)
    + (-2.0716198*pow(10, -2)*pow(R, 2)) + (6.87678*pow(10, -4))*pow(T, 2)*R + (2.74954*pow(10, -4))*T*pow(R, 2)
    + (0*pow(10, -6))*pow(T, 2)*pow(R, 2);
  return HI-1;
}

//led color manager
void setColor(int r, int g, int b)
{
  #ifdef COMMON_ANODE
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  #endif
  analogWrite(ledR, r);
  analogWrite(ledG, g);
  analogWrite(ledB, b);
}

//sets led color corresponding to Heat Index value
//using led color manager 'setColor'
void setLEDHI(int HI)
{ 
  if(HI>=115)
  {
    //red
    setColor(255, 165, 0);
  }

  else if(HI>=103)
  {
    //orange
    setColor(255, 165, 0);
  }

  else if (HI>=91)
  {
    //yellow
    setColor(255, 255, 0);
  }
  else if (HI<91)
  {
    //green
    setColor(0, 255, 0);
  }
}

/* HBD SONG FUNCTIONS */
//=====================//
void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
     digitalWrite(speakerPin, HIGH);
     delayMicroseconds(tone);
     digitalWrite(speakerPin, LOW);
     delayMicroseconds(tone);
 }
}

 void playNote(char note, int duration) {
  char names[] = {'C', 'D', 'E', 'F', 'G', 'A', 'B',           
                   'c', 'd', 'e', 'f', 'g', 'a', 'b',
                   'x', 'y' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014,
                   956,  834,  765,  593,  468,  346,  224,
                   655 , 715 };
  int SPEE = 5;
  // play the tone corresponding to the note name
  for (int i = 0; i < 17; i++) {
     if (names[i] == note) {
      int newduration = duration/SPEE;
       playTone(tones[i], newduration);
   }
  }
 }

void singHBD(){
  int length = 28; // the number of notes
  char notes[] = "GGAGcB GGAGdc GGxecBA yyecdc";
  int beats[] = { 2, 2, 8, 8, 8, 16, 1, 2, 2, 8, 
                8,8, 16, 1, 2,2,8,8,8,8,16, 1,
                2,2,8,8,8,16 };
  int tempo = 150;
  for (int i = 0; i < length; i++) {
    if (notes[i] == ' ') delay(beats[i] * tempo); // rest
    else playNote(notes[i], beats[i] * tempo);
   
    // pause between notes
   delay(tempo);
  }
}
//=====================//
//=====================//

