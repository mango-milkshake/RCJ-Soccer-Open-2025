#ifndef __ANALOG_IIC_H
#define __ANALOG_IIC_H
#include <Arduino.h>
#include "stdint.h"

#define Delay_us delayMicroseconds
// #define scl_pin	1
// #define sda_pin	0


#define SDA_Dout_LOW(sda_pin) digitalWrite(sda_pin,LOW)
#define SDA_Dout_HIGH(sda_pin) digitalWrite(sda_pin,HIGH)
#define SDA_Data_IN(sda_pin) digitalRead(sda_pin)
#define SCL_Dout_LOW(scl_pin) digitalWrite(scl_pin,LOW)
#define SCL_Dout_HIGH(scl_pin) digitalWrite(scl_pin,HIGH)
#define SCL_Data_IN(scl_pin) digitalRead(scl_pin)
#define SDA_Write(sda_pin, XX) digitalWrite(sda_pin,(XX?HIGH:LOW))

void Analog_IIC_Init(uint8_t scl_pin, uint8_t sda_pin);
void Analog_IIC_Start(uint8_t scl_pin, uint8_t sda_pin);
void Analog_IIC_Stop(uint8_t scl_pin, uint8_t sda_pin);
void Analog_IIC_Send_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t txd);
uint8_t Analog_IIC_Read_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t ack);
void Analog_IIC_NAck(uint8_t scl_pin, uint8_t sda_pin);
void Analog_IIC_Ack(uint8_t scl_pin, uint8_t sda_pin);
uint8_t Analog_IIC_Wait_Ack(uint8_t scl_pin, uint8_t sda_pin);


#endif


