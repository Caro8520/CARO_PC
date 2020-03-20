// tabla={0x7E,0X30,0X6D,0X79,0X33,0X5B,0X5F,0X70,0X7F,0X7B};
#define Pi 3.14159
#define T  0.005 
const bool tabla[10][7]={{1,1,1, 1,1,1,0},
                         {0,1,1, 0,0,0,0},
                         {1,1,0, 1,1,0,1},
                         {1,1,1, 1,0,0,1},
                         {0,1,1, 0,0,1,1},
                         {1,0,1, 1,0,1,1},
                         {1,0,1, 1,1,1,1},
                         {1,1,1, 0,0,0,0},
                         {1,1,1, 1,1,1,1},
                         {1,1,1, 1,0,1,1}};
//-------------pines----------------//                         
const uint8_t Display[7]={4,5,6,7,8,3,2}; // pines display
const uint8_t hab[3]={9,10,11};// pines habilitadores 
const uint8_t pulso=14;// pin pulso  
//--------variables ---------------//
unsigned long tiempo=0, tiempo1=0, tiempoCambios;  
uint8_t numDis=0;
uint8_t digitos[3]={0,0,0};  
int filtro[4]={0,0,0,0};
int filtro_pro[2]={0,0};
char x[3]={0,0,0};
int j,k=0;
int dato=0;
int cruce_cero=1200;
int cuentas=15;
unsigned long promedio=0;
int promedioF=0;
int cuenta_prom=0;
unsigned long periodo=0,periodo1=0;
float frecuencia=0;
int n=0;
int voltaje=0,derivada=0;
void setup() {
 Serial.begin(115200);
 
 for(int i=0;i<7;i++){
  pinMode(Display[i],OUTPUT);
 }
 
 for(int i=0;i<3;i++){
  pinMode(hab[i],OUTPUT);
 }
 tiempo=micros();
 digitalWrite(hab[0],HIGH);
 digitalWrite(hab[1],LOW);
 digitalWrite(hab[2],LOW);

  
}

void loop() {
  //-------------multiplexacion digitos-----------//
  visualizacion();

  
 //---------------cambio de valor-----------------//
  /*if(micros()-tiempo1>=1000000){
      tiempo1=micros();
      digitos[0]++;
      digitos[1]++;
      digitos[2]++;
      if(digitos[0]>9){
        digitos[0]=0;
        digitos[1]=0;
        digitos[2]=0;
      }
   }*/
}
  
  
void visualizacion(){
  if(micros()-tiempo>=5000){
    tiempo=micros();
    filtrar();
    frecuenciaPPM();
    if(cuenta_prom%1==0){
//      Serial.print(filtro_pro[1]);
//      Serial.print(',');      
      //Serial.print(promedioF);
      //Serial.print(',');
      Serial.print(cruce_cero);
      Serial.print(',');
      Serial.println(filtro[2]);     
    }
    if(cuenta_prom%2==0){
      separarDigitos();
    }
    imp7segmentos();
  }
}

void separarDigitos(){
  char c[3]={0,0,0};
  sprintf(c,"%d",(int)(frecuencia));
  if(frecuencia==0){
    digitos[0]=0;
    digitos[1]=0;
    digitos[2]=0;
  }else{
    digitos[0]=(int)c[0] -'0';
    digitos[1]=(int)c[1] -'0';
    digitos[2]=(int)c[2] -'0';
  }
}

void filtrar(){
    filtro[0]=filtro[1];
    filtro[1]=filtro[2];
    filtro[2]=filtro[3];
    voltaje=analogRead(14)*(5000/1023);
    filtro[3]=(voltaje+filtro[0]+filtro[1]+filtro[2])/4;
    filtro_pro[0]=filtro_pro[1];
    filtro_pro[1]=(0.03*Pi*filtro[2]+filtro_pro[0])/(1+0.03*Pi);
//    promedioF=1100; 
    derivada=(filtro_pro[1]-filtro_pro[0]);
    promedio=promedio+filtro[2];
    cuenta_prom++;
    if(cuenta_prom>200){
     promedioF=promedio/cuenta_prom;
     cuenta_prom=0;
     promedio=0;
    }
}

void imp7segmentos(){
    numDis++;
    if(numDis>2){
      numDis=0;
    }
    digitalWrite(hab[0],LOW);
    digitalWrite(hab[1],LOW);
    digitalWrite(hab[2],LOW);
    digitalWrite(hab[numDis],HIGH);
    for(int i=0;i<7;i++){
      digitalWrite(Display[i],tabla[digitos[numDis]][i]);
    }
}

void frecuenciaPPM(){
    if(micros()-tiempoCambios>2000000){
       frecuencia=0;
       frecuencia=622;
    }
    if(cuentas==20){
      if(filtro[3]>=1250 && filtro[3]<=1350){
        tiempoCambios=micros();
        if(cruce_cero==1200){
          cruce_cero=1000;
        }else{
          n++;
          if(n>=3){
            periodo1=periodo;
            periodo=micros();
            //frecuencia=1500000000.0/(periodo-periodo1);// 1000000*60/4    
            frecuencia=622;
            if(frecuencia<30){
              frecuencia=0;
              frecuencia=622;
            }
            n=0;
          }
          cruce_cero=1200; 
        }
        cuentas=19;
      }
    }else{
      cuentas=cuentas-1;
      if(cuentas==0){
        cuentas=20;
      }
    }
}
