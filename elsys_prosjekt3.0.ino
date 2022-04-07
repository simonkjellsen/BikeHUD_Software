#include <MPU9250_asukiaaa.h>
bool blinkR=false;
bool blinkL = false;

int ledR=14;
int ledL=15;

const long interval=250;
unsigned long prev_mil=0;
unsigned long blink_mil=0;

int RedR = 255;
int RedL = 255;

int gyro_mil=0;
int count=0;
bool gyro = false;

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;
float aZ;
bool getAccelZ();

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif
  mySensor.beginAccel();

pinMode(26,INPUT);
pinMode(ledR, OUTPUT);
pinMode(25,INPUT);
pinMode(ledL, OUTPUT);

pinMode(13,OUTPUT); //HIGH KNAPP
digitalWrite(13,HIGH);

pinMode(12,OUTPUT); //HIGH KNAPP
digitalWrite(12,HIGH);

pinMode(32,OUTPUT); //HIGH LED
digitalWrite(32,HIGH);

pinMode(27,OUTPUT); //HIGH LED
digitalWrite(27,HIGH);

pinMode(18,OUTPUT); //GROUND KNAPP
digitalWrite(18,LOW);
pinMode(19,OUTPUT); //GROUND KNAPP
digitalWrite(19,LOW);

}

void loop() {
  
  uint8_t sensorId;
if (digitalRead(25)==HIGH){
  if (blinkL){
    blinkL=false;
  }
  blinkR=!blinkR;
  delay(200);}
if (blinkR && ((millis()-prev_mil)>interval)){    //&& (millis()%1000==0)){

prev_mil=millis();
  if (RedR==255){
    RedR=0;
  digitalWrite(ledR,HIGH);
  }
  else{
    RedR=255;
  digitalWrite(ledR,LOW);
  
  }
}
if(!blinkR){
  digitalWrite(ledR,HIGH);  
}

//left led
if (digitalRead(26)==HIGH){
  if (blinkR){
    blinkR=false;
  }
  blinkL=!blinkL;
  delay(200);}
  
if (blinkL && ((millis()-prev_mil)>interval)){    //&& (millis()%1000==0)){

prev_mil=millis();
  if (RedL==255){
    RedL=0;
  digitalWrite(ledL,LOW);
  
  }
  else{
    RedL=255;
  digitalWrite(ledL,HIGH);

  }
}
if(!blinkL){
  digitalWrite(ledL,HIGH);
  }

//Gyroskop
if (millis()-gyro_mil>1000){
  gyro_mil=millis(); 
  if (getAccelZ()){
    Serial.print("getaccel");
    count++;
    gyro=true;
  }
  if (!getAccelZ()&&(count>0)){
    Serial.print("ikke getaccel");
    blinkR=false;
    blinkL=false;
    count=0;
  }
  
}
if (gyro&&(blinkR||blinkL)){
  if (!getAccelZ()){
    Serial.print("Gyro");
    blinkR=false;
    blinkL=false;
    gyro = false;
    count = 0;
  }
}  
}

//Funksjon som heter accelZ
bool getAccelZ(){
  if (mySensor.accelUpdate() == 0) {
    aZ = mySensor.accelZ();
    Serial.println("accelZ: " + String(aZ));
  
  } else {
    Serial.println("Cannod read accel values");
  }
    if(aZ > -2.40){
      return true;
    }
    else {
      return false;
      }
  }
