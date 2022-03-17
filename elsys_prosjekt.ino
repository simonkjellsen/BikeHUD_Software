bool blinkR=false;
bool blinkL = false;

//bool on=false;
int redPinR=3;
int greenPinR=5;
int bluePinR=6;

int redPinL=9;
int greenPinL=10;
int bluePinL=11;

const long interval=200;
unsigned long prev_mil=0;
unsigned long blink_mil=0;

int RedR = 255;
int RedL = 255;
void setup() {
  // put your setup code here, to run once:

pinMode(8,INPUT);
pinMode(redPinR, OUTPUT);
pinMode(bluePinR, OUTPUT);
pinMode(greenPinR, OUTPUT);


pinMode(13,INPUT);
pinMode(redPinL, OUTPUT);
pinMode(bluePinL, OUTPUT);
pinMode(greenPinL, OUTPUT);


}

void loop() {
//for( i=0; i<1000 ; i++){
if (digitalRead(8)==HIGH){
  if (blinkL){
    blinkL=false;
  }
  blinkR=!blinkR;
  delay(200);}

//}


  
if (blinkR && ((millis()-prev_mil)>interval)){    //&& (millis()%1000==0)){

prev_mil=millis();
  if (RedR==255){
    RedR=0;
  analogWrite(redPinR,0);
  analogWrite(greenPinR,220);
  analogWrite(bluePinR,255);
  }
  else{
    RedR=255;
  analogWrite(redPinR,255);
  analogWrite(greenPinR,255);
  analogWrite(bluePinR,255);
  }
  
//on=true;
//delay(5);
}
if(!blinkR){
  analogWrite(redPinR,255);
  analogWrite(greenPinR,255);
  analogWrite(bluePinR,255);
}



//}

//left led
if (digitalRead(13)==HIGH){
  if (blinkR){
    blinkR=false;
  }
  blinkL=!blinkL;
  delay(200);}
  
if (blinkL && ((millis()-prev_mil)>interval)){    //&& (millis()%1000==0)){

prev_mil=millis();
  if (RedL==255){
    RedL=0;
  analogWrite(redPinL,0);
  analogWrite(greenPinL,220);
  analogWrite(bluePinL,255);
  }
  else{
    RedL=255;
  analogWrite(redPinL,255);
  analogWrite(greenPinL,255);
  analogWrite(bluePinL,255);
  }
  
//on=true;
//delay(5);
}
if(!blinkL){
  analogWrite(redPinL,255);
  analogWrite(greenPinL,255);
  analogWrite(bluePinL,255);
}




 
}
