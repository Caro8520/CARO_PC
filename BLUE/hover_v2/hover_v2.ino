#define M1 5
#define M2 6
#define M1_L1 3
#define M1_L2 4
#define M2_L1 8
#define M2_L2 7
bool inicio=0;
int pos=0;
int pwm_max=190;
int pwm_min=120;
void setup() {
  pinMode(4,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  analogWrite(M1,0);
  analogWrite(M2,0);
  digitalWrite(M1_L1,HIGH);
  digitalWrite(M1_L2,LOW);
  digitalWrite(M2_L1,HIGH);
  digitalWrite(M2_L2,LOW);
  Serial.begin(9600);
}
void loop() {
  if (Serial.available()>0){
    String entrada = Serial.readStringUntil('\n');
    Serial.println(entrada);
    if(entrada=="A"){
      inicio=1;
    }else if(entrada=="B"){
      inicio=0;
    }else if(entrada.charAt(0)=='C'){
      pwm_max=entrada.substring(1,entrada.length()).toInt();
    }else if(entrada.charAt(0)=='D'){
      pwm_min=entrada.substring(1,entrada.length()).toInt();
    }else if (entrada >= "1" && entrada <= "4"){
      pos=entrada.toInt();
    }
  }
  if(inicio==1){
    switch (pos) {
      case 1:
        analogWrite(M1,pwm_max);
        analogWrite(M2,pwm_max);
        break;
      case 2:
        analogWrite(M1,pwm_min);
        analogWrite(M2,pwm_min);
        break;
      case 3:
        analogWrite(M1,pwm_max);
        analogWrite(M2,pwm_min);
        break;
      case 4:
        analogWrite(M1,pwm_min);
        analogWrite(M2,pwm_max);
        break;
      default:
        analogWrite(M1,pwm_min);
        analogWrite(M2,pwm_min);
        break;
    }
    
  }else{
    analogWrite(M1,pwm_min);
    analogWrite(M2,pwm_min);
  }

}
