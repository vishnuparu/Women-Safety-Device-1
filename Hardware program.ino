#define fsrpin A0 //FSR is connected to analog 0

#define PulseWire A1//pulse sensor is connected to analog 1

#define USE_ARDUINO_INTERRUPTS true

#include<PulseSensorPlayground.h>

#include<TinyGPS.h>

#include <SoftwareSerial.h>

int state=0;//state of gps

const int pin= 9;

float gpslat,gpslon;

int fsrreading;//analog reading from the FSR

//conditions which turn the device ON and OFF

boolean condition1;

boolean condition2;

boolean condition3;

int Button1 =2;//button1 is connected to pin2

int Button2=5;//button1 is connected to pin5

int LED =13;//inbuilt LED

int buzzer=9;//Buzzer connected to pin9

//initial conditions

int stateLED=LOW;//state of LED

int statebuzzer=LOW;//state of Buzzer

int stateshock=0;//state of shock

int stateButton1;//state of button1

int stateButton2;//state of button2

//Determine which signal to count as a heart beat

int Threshold=550;

TinyGPS gps;

SoftwareSerial sgps(4,5);

SoftwareSerial sgsm(2,3);

PulseSensorPlayground pulseSensor;


void setup(){
  
  pinMode(Button1,INPUT);
  
  pinMode(Button2,INPUT);
  
  pinMode(LED,OUTPUT);
  
  pinMode(buzzer,OUTPUT);
  
  pinMode(fsrpin,INPUT);
  
  Serial.begin(9600);
  
  sgsm.begin(9600);
  
  sgps.begin(9600);
  
  pulseSensor.analogInput(PulseWire);
  
  pulseSensor.setThreshold(Threshold);
 }


 void loop(){
  
  fsrreading =analogRead(fsrpin);//read the analog value from FSR
  
  Serial.print("The force sensor reads:");
  
  Serial.println(fsrreading);
  
  delay(500);

  //calculate the heart beat per minute
  
  int myBPM =pulseSensor.getBeatsPerMinute();
  
  if (pulseSensor.sawStartOfBeat()){
    
    Serial.print("Her beat per minute is:");
    
    Serial.println(myBPM);
    
    }
    
  else{
    
    Serial.println("The pulse sensor is not sensing any heartbeat");
    
    Serial.println("BPM is 0");
    
  }
  delay(500);  
  
  stateButton1=digitalRead(Button1);
  
  stateButton2=digitalRead(Button2);

  //condition for the device to be turned ON
  
  condition1 = stateButton1 == HIGH ;

  //condition which the FSR and pulse sensor is above threshold
  
  condition2 = fsrreading>300 && myBPM > 90;

  //condition for the device to be turned off
  
  condition3 = stateButton2 == HIGH;
  
  sgps.listen();
  
  while (sgps.available()){
    
    int c=sgps.read();
    
    if (gps.encode(c)){
      
      gps.f_get_position(&gpslat,&gpslon);
      
    }
    
  }

  
  //check if any of the condition is true
  
  if((condition1 || condition2) && state == 0){

    statebuzzer=HIGH;
    
    stateLED=HIGH;
    
    stateshock=1;
    
    digitalWrite(LED,stateLED);
    
    digitalWrite(buzzer,statebuzzer);
    
    Serial.println("The alert system is now ACTIVATED!!!");
    
    sgsm.listen();
    
    sgsm.print("/r");
    
    delay(1000);
    
    sgsm.print("AT+CMGF=1\r");
    
    delay(1000);
    
    sgsm.print("AT+CMGS=\"+919746980135\"\r");
    
    delay(1000);
    
    sgsm.print("latitude :");
    
    sgsm.println(gpslat,6);
    
    sgsm.print("longitude:");
    
    sgsm.println(gpslon,6);
    
    delay(1000);
    
    state=1;
    
    
    
    //shock generator
    
    if (stateshock==1){
      
    digitalWrite(13,HIGH);
    
    digitalWrite(12,LOW);
    
    delayMicroseconds(500);
    
    digitalWrite(13,LOW);
    
    digitalWrite(12,HIGH);
    
    delayMicroseconds(500);
    }
    
  }
  
  
  if(condition3){
    
    statebuzzer=LOW;
    
    stateLED=LOW;
    
    stateshock=0;
    
    digitalWrite(LED,stateLED);
    
    digitalWrite(buzzer,statebuzzer);
    
    state=0;
    
    Serial.println("The system is now deactivated!!!!");
    
  }
  
 }
