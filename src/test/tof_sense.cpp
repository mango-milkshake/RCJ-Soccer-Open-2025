#include "analog_iic.h"
#include "tofsense_f_iic.h"
// #include <Wire.h>

tofsense_f_output_parameter tf1, tf2, tf3, tf4, tf5, tf6, tf7, tf8;
uint8_t addr[9];
uint8_t scl_pin = 1, sda_pin = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  // Wire.setSCL(1);
  // Wire.setSDA(0);
  // Wire.begin(); 
  // Wire.setClock(400000); 

  for (int i=0; i<=8; i++){
    addr[i] = 0x08+i;
  }

  Analog_IIC_Init(scl_pin, sda_pin);
  
}

void loop() {

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[1],&tf1);
  Serial.print("1 ");
  Serial.print(tf1.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[2],&tf2);
  Serial.print(" 2 ");
  Serial.print(tf2.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[3],&tf3);
  Serial.print(" 3 ");
  Serial.print(tf3.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[4],&tf4);
  Serial.print(" 4 ");
  Serial.print(tf4.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[5],&tf5);
  Serial.print(" 5 ");
  Serial.print(tf5.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[6],&tf6);
  Serial.print(" 6 ");
  Serial.print(tf6.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[7],&tf7);
  Serial.print(" 7 ");
  Serial.print(tf7.dis);

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,addr[8],&tf8);
  Serial.print("8 ");
  Serial.print(tf8.dis);

  Serial.println();
  
  // delay(10);

  
}
