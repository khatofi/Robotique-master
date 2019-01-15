#include <ZigduinoRadio.h>
#define MOVE 4
int value=0;
void setup(){
  pinMode(MOVE,INPUT);
  ZigduinoRadio.setChannel(26);
  ZigduinoRadio.begin(26);
  Serial.begin(9600);
  ZigduinoRadio.beginTransmission();
  ZigduinoRadio.attachError(errHandle);
  ZigduinoRadio.attachTxDone(onXmitDone);
}

void loop(){
    value=digitalRead(MOVE);
    ZigduinoRadio.beginTransmission();
    Serial.println();
    Serial.print("Tx: ");
    //Serial.println(value);
    if(value==1){
      Serial.write('1');
      ZigduinoRadio.write('1');
    } 
    else if(value==0){
      Serial.write('0');
      ZigduinoRadio.write('0');
    }         
    Serial.println();    
    ZigduinoRadio.endTransmission();
    delay(1000);
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



