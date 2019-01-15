/*

Run this sketch on two Zigduinos, open the serial monitor at 9600 baud, and type in stuff
Watch the Rx Zigduino output what you've input into the serial port of the Tx Zigduino

*/

#include <ZigduinoRadio.h>

#define OFF '2'
#define ON '3'
#define LEDON '4' 

#define PIROFF '0'
#define PIRON '1'


#define led 2
char val = '0',valmotion = '0';
//Variable eteint pour stocker l'etat du led (Si il est allume par la gateway il prend la valeur ledon , si il est eteint par la gateway il s'allume et s'eteint suivant les valeurs du PIR) 
char eteint = LEDON;
void setup()
{
  pinMode(led, OUTPUT);
  ZigduinoRadio.begin(26);
  Serial.begin(9600);
  
  //ZigduinoRadio.attachError(errHandle);
 // ZigduinoRadio.attachTxDone(onXmitDone);
  digitalWrite(led,LOW);
}

void loop()

{


  
  if (Serial.available())
  {
    ZigduinoRadio.beginTransmission();
    
    //Serial.println();
    //Serial.print("Tx: ");
    
    while(Serial.available())
    {
      char c = Serial.read();
      Serial.write(c);
      ZigduinoRadio.write(c);
    }
    
    //Serial.println(); 

    
    
    ZigduinoRadio.endTransmission();
  }
  
  if (ZigduinoRadio.available())
  {

    while(ZigduinoRadio.available())
    {
      val = ZigduinoRadio.read(); 
      Serial.write(val);

      //Allumer la lumière via la gateway depuis la commande dans le terminal 
     /* if (val == ON )
      { digitalWrite(led,HIGH);
        //Serial.write("LED ON");
      //la variable eteint est mise sur LEDON pour que la LED  soit toujours allumee quoi que ce soit la valeur du PIR 
      eteint = LEDON;
      }
      // Eteindre la lumière via la gateway 
      else if (val == OFF )
      
      { digitalWrite(led,LOW);
        //Serial.write("LED OFF");
        //Cette fois-ci il faut anticiper si il y'a  mouvement detecté par le PIR, et le faut stocker l'etat du led 'eteint'
        eteint = PIRON;
      }

      else if(val == PIRON && eteint == PIRON){
        digitalWrite(led,HIGH);
        //Serial.write("Mouvement!");
        eteint = PIROFF;
      }
      else if(val == PIROFF && eteint == PIROFF){
        digitalWrite(led,LOW);
        eteint = PIRON;
      }*/
      Serial.println();
     
    }
  } 
  /*else
  {
     digitalWrite(led,LOW); 
  }*/
  
  delay(100);
}

void errHandle(radio_error_t err)
{
  Serial.println();
  Serial.print("Error: ");
  Serial.print((uint8_t)err, 10);
  Serial.println();
}

void onXmitDone(radio_tx_done_t x)
{
  Serial.println();
  Serial.print("TxDone: ");
  Serial.print((uint8_t)x, 10);
  Serial.println();
}
