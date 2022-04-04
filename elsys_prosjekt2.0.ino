#include <MPU9250_asukiaaa.h>
bool blinkR=false;
bool blinkL = false;

int redPinR=15;
int redPinL=14;

const long interval=250;
unsigned long prev_mil=0;
unsigned long blink_mil=0;

int RedR = 255;
int RedL = 255;

bool gyro_1s=false; 
bool gyro_rn=false; 
int gyro_mil=0;
int count=0;
//bool gyro = false;

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
pinMode(redPinR, OUTPUT);
pinMode(25,INPUT);
pinMode(redPinL, OUTPUT);

}

void loop() {
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
  digitalWrite(redPinR,HIGH);
  }
  else{
    RedR=255;
  digitalWrite(redPinR,LOW);
  
  }
}
if(!blinkR){
  digitalWrite(redPinR,HIGH);  
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
  digitalWrite(redPinL,LOW);
  
  }
  else{
    RedL=255;
  digitalWrite(redPinL,HIGH);

  }
}
if(!blinkL){
  digitalWrite(redPinL,HIGH);
  }

//Gyroskop
if (millis()-gyro_mil>1000){
  gyro_mil=millis(); 
  //gyro_1s=digitalRead(gyro); 
  if (getAccelZ()){
    count++;
    //gyro=true;
  }
  if (!getAccelZ()&&(count>0)){
    blinkR=false;
    blinkL=false;
    count=0;
  }
  
}
if (getAccelZ()&&(blinkR||blinkL)){
  //gyro_rn=digitalRead(gyro);
  if (!getAccelZ()){
    blinkR=false;
    blinkL=false;
  }
}
}

bool getAccelZ(){
   uint8_t sensorId;
   if (mySensor.readId(&sensorId) == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId");
  }

  if (mySensor.accelUpdate() == 0) {
    aZ = mySensor.accelZ();
    Serial.println("accelZ: " + String(aZ));
    
  } else {
    Serial.println("Cannod read accel values");
  } 
    if(aZ < -2.40){
      return true;
    }
    else {
      return false;
      }
  Serial.println("at " + String(millis()) + "ms");
  Serial.println(""); // Add an empty line
  delay(1000);
  }
