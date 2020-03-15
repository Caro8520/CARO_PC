

void setup(){
  pinMode(4,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
}

void loop(){
   digitalWrite(4,HIGH);
   digitalWrite(3,LOW);
   digitalWrite(7,HIGH);
   digitalWrite(8,LOW);
   analogWrite(5,190); // Señal PWM a 75% en el PIN 6
   analogWrite(6,190); // Señal PWM a 75% en el PIN 6
}
