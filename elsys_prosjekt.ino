bool blink=false;
void setup() {
  // put your setup code here, to run once:

pinMode(14,INPUT);
pinMode(33, OUTPUT);

}

void loop() {

if (digitalRead(14)==HIGH){
  blink=!blink;}
if (blink){
digitalWrite(33,HIGH); 
delay(500);
digitalWrite(33,LOW);
delay(500);
}

 
}
