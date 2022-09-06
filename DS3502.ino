
#include <Wire.h>

//アドレス指定
#define DS3502_I2CADDR_0  0x28
#define DS3502_I2CADDR_1  0x2B
#define DS3502_WIPER      0x00  ///< Wiper value register
#define DS3502_MODE       0x02  ///< Mode selection register

#define Serialport SerialUSB

float temp;
uint32_t data,status,i;
uint16_t temp_hex,counter_a;
uint8_t L_tmp, R_tmp;
uint8_t read_data[14];

void Init_DS3502(uint32_t address){
  if(address == 0){
    Wire.beginTransmission(DS3502_I2CADDR_0);
  }
  else if(address == 1){
    Wire.beginTransmission(DS3502_I2CADDR_1);
  }
  data = Wire.endTransmission();
  Serialport.println(data);

  control_data_str.Lch_wiper_position = 127;
  control_data_str.Rch_wiper_position = 127;
}

uint32_t Write_DS3502(uint32_t address, int8_t data_){
  uint8_t address_,tmp_;

  if(address == 0){
    address_ = DS3502_I2CADDR_0;
  }
  else if(address == 1){
    address_ = DS3502_I2CADDR_1;
  }
  Wire.beginTransmission(address_);
  Wire.write(DS3502_WIPER);
  Wire.write(data_);
  
  return Wire.endTransmission();
}

uint8_t Read_DS3502(uint32_t address){
  uint8_t address_,tmp_,data_;

  if(address == 0){
    address_ = DS3502_I2CADDR_0;
  }
  else if(address == 1){
    address_ = DS3502_I2CADDR_1;
  }
  Wire.beginTransmission(address_);
  Wire.write(DS3502_WIPER);
  tmp_ = Wire.endTransmission();

  Wire.requestFrom(address_, 1);
  data_ = Wire.read();
  tmp_ = Wire.endTransmission();

  return data_;
}

uint8_t setGain(float x, float y){
  L_tmp = (uint8_t)(120 * x / (float)ROOM_L); //* (3 - y) / (float)ROOM_H) + 7; 
  R_tmp = (uint8_t)(120 * ((float)ROOM_L - x) / (float)ROOM_L); // * (3 - y) / (float)ROOM_H) + 7;
  control_data_str.Lch_wiper_position = (uint8_t)L_tmp; //(L_tmp * (-0.2 * y + 1));
  control_data_str.Rch_wiper_position = (uint8_t)R_tmp; //(R_tmp * (-0.2 * y + 1));
}


void ds3502_task(void){
  uint8_t Lch_wiper_position,Rch_wiper_position;
  Lch_wiper_position = control_data_str.Lch_wiper_position;
  Rch_wiper_position = control_data_str.Rch_wiper_position;
  status = Write_DS3502(0,Lch_wiper_position);
  delay(16);
  status = Write_DS3502(1,Rch_wiper_position);
  delay(16);
  delay(100);
}
