#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  
SoftwareSerial BT(10,11);    // Definimos los pines RX y TX del Arduino conectados al Bluetooth
 
void setup()
{
  Serial.begin(230400);       // Inicializamos el puerto serie BT (Para Modo AT 2)
  //Serial.begin(9600);   // Inicializamos  el puerto serie  
}
 
void loop()
{
  if(Serial.available())  // Si llega un dato por el monitor serial se envía al puerto BT
  {
     String in=Serial.readStringUntil('\n');
     Serial.println(in);
  }
}
