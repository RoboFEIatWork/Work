#include <ArduinoJson.h>
#include <stdio.h>

int ledAzul = 2;


void setup() {
  Serial.begin(9600);
  pinMode(ledAzul, OUTPUT); 
  while(!Serial) {
  }
}

void loop() {
  int     size_ = 0;
  String  payload;
  while ( !Serial.available()  ){}
  if ( Serial.available() )
    payload = Serial.readStringUntil( '\n' );
  StaticJsonDocument<512> doc;

  DeserializationError   error = deserializeJson(doc, payload);
  if (error) {
    Serial.println(error.c_str()); 
    return;
  }
  if (doc["operation"] == "sequence") {
     digitalWrite(ledAzul,HIGH);
  }
  else {
      digitalWrite(ledAzul,LOW);
   }
  delay(20);
}
