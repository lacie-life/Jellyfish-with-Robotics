#include <SoftwareSerial.h>
SoftwareSerial ArduinoUno(3,2);
int led = 13;


void setup(){
	
	Serial.begin(9600);
	ArduinoUno.begin(9600);
  pinMode(led, OUTPUT); 

}

void loop(){
	
	while(ArduinoUno.available()>0){
	  char val = ArduinoUno.read();
    Serial.println(val);
    if (val == '1'){
      digitalWrite(led, HIGH);
    }
    else if (val == '0'){
      digitalWrite(led, LOW);
    }
  }
  delay(30);
}
