#define Pulse 7

#define Dir 6

long delay_Micros =1800; // Set value

long currentMicros = 0; long previousMicros = 0;

void setup(){

  pinMode(Pulse,OUTPUT);

  pinMode(Dir,OUTPUT);
  digitalWrite(Dir,LOW);

}

void pwmSend(int rate, int dir){
  digitalWrite(Dir,dir);
  
  digitalWrite(Pulse,HIGH);
  delayMicroseconds(rate); //Set Value

  digitalWrite(Pulse,LOW);
  delayMicroseconds(rate);
}

void loop(){
  pwmSend(1000, 1);
  
 }
