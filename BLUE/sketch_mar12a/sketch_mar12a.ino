bool estado=0;
void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}
void loop() {
  if (Serial.available()>0){
    if(estado==1){estado=0;}
    else{estado=1;}
    String entrada = Serial.readStringUntil('\n');
    Serial.println(entrada);
    digitalWrite(13,estado);
  }

}
