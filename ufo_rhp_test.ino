#include <Wire.h>
#include <stdlib.h>
#include <util/delay.h>

#define slave 0x55

#define data_ready_req 0x10
#define data_value_req 0x20
#define sensor_nval_req 0x30
#define debug_nval_req 0x40
#define sensor_sid_req 0x50
#define debug_sid_req 0x60
#define data_bytes_req 0x70
#define trigger_req 0x80

float dummy_float = 0;
float dummy_int = 0;
byte dummy_byte = 0;
uint8_t * return_byte_array; 
uint8_t * data_value_array; 
uint8_t sensor_nval = 0;
uint8_t debug_nval = 0;
uint8_t data_bytes = 0;
uint8_t data_ready = 0;
uint8_t * sensor_sid_array;
uint8_t * debug_sid_array;

char buffer[20];
char *names[] = {"rh", "t"};

void setup(){
  Wire.begin();
  Serial.begin(9600); 
  Serial.println("Serial Begin");

  Wire.beginTransmission(slave);
  Wire.write(sensor_nval_req);
  Wire.endTransmission();
  Wire.requestFrom(slave, 1);
  sensor_nval = (uint8_t) Wire.read();
  sensor_sid_array = (uint8_t *) calloc(sensor_nval, sizeof(uint8_t));
  Serial.print("Sensor Nval: "); 
  Serial.println(sensor_nval); 

  Wire.beginTransmission(slave);
  Wire.write(debug_nval_req);
  Wire.endTransmission();
  Wire.requestFrom(slave, 1);
  debug_nval = (uint8_t) Wire.read();
  debug_sid_array = (uint8_t *) calloc(debug_nval, sizeof(uint8_t));
  Serial.print("Debug Nval:  "); 
  Serial.println(debug_nval); 
  
  Wire.beginTransmission(slave);
  Wire.write(data_bytes_req);
  Wire.endTransmission();
  Wire.requestFrom(slave, 1);
  data_bytes = (uint8_t) Wire.read();
  return_byte_array = (uint8_t *) calloc(data_bytes, sizeof(uint8_t));
  Serial.print("Data Bytes:  "); 
  Serial.println(data_bytes); 
  
  data_value_array = (uint8_t *) calloc(debug_nval, data_bytes);
  
  Wire.beginTransmission(slave);
  Wire.write(sensor_sid_req);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)slave, sensor_nval);
  Serial.print("Sensor SID: "); 
  for (int iS = 0; iS<sensor_nval; iS++){
    sensor_sid_array[iS] = (uint8_t) Wire.read();
    Serial.print("  ");Serial.print(sensor_sid_array[iS], HEX); 
  }
  Serial.print("\r\n");

  Wire.beginTransmission(slave);
  Wire.write(debug_sid_req);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)slave, debug_nval);
  Serial.print("Debug SID:  "); 
  for (int iS = 0; iS<debug_nval; iS++){
    debug_sid_array[iS] = (uint8_t) Wire.read();
    Serial.print("  ");Serial.print(debug_sid_array[iS], HEX); 
  }
  Serial.print("\r\n");

}

void loop(){

  Wire.beginTransmission(slave);
  Wire.write(trigger_req);
  Wire.endTransmission();

  while (true){
    Wire.beginTransmission(slave);
    Wire.write(data_ready_req);
    Wire.endTransmission();
    Wire.requestFrom(slave, 1);
    data_ready = (uint8_t) Wire.read();
    if (data_ready == 1) break;
    _delay_ms(50);
  }

  for (int iDV = 0; iDV<debug_nval*data_bytes; iDV++){
   data_value_array[iDV] = 0; 
  }


  for (int iS = 0; iS<debug_nval; iS++){    
    Wire.beginTransmission(slave);
    Wire.write(data_value_req);
    Wire.write(debug_sid_array[iS]);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)slave, data_bytes);
    for (int iB = 0; iB<data_bytes; iB++){
      return_byte_array[iB] = (uint8_t) Wire.read();
    }    
    memcpy(&data_value_array[iS*data_bytes], return_byte_array, data_bytes);
    Serial.print("Sensor SID: "); Serial.print(debug_sid_array[iS],HEX); Serial.print(" "); Serial.print(names[iS]); Serial.print(" Datum ");
    switch(data_bytes){
      case 4:
        memcpy(&dummy_float, &data_value_array[iS*data_bytes], data_bytes);
        dtostrf((double)dummy_float, 20, 4, buffer);
        Serial.println(buffer);
        break;
      case 2:
        memcpy(&dummy_int, &data_value_array[iS*data_bytes], data_bytes);
        Serial.println(dummy_int);
        break;
      case 1:
        memcpy(&dummy_byte, &data_value_array[iS*data_bytes], data_bytes);
        Serial.println(dummy_byte);
        break;
    }      
  }  
  Serial.println("");
  delay(5000);

}







