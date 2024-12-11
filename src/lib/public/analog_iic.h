#ifndef __ANALOG_IIC_H
#define __ANALOG_IIC_H
#include <Arduino.h>
#include "stdint.h"

#define Delay_us delayMicroseconds
#define ANALOG_IIC_SCL_PIN	5
#define ANALOG_IIC_SDA_PIN	4


#define SDA_Dout_LOW() digitalWrite(ANALOG_IIC_SDA_PIN,LOW)
#define SDA_Dout_HIGH() digitalWrite(ANALOG_IIC_SDA_PIN,HIGH)
#define SDA_Data_IN() digitalRead(ANALOG_IIC_SDA_PIN)
#define SCL_Dout_LOW() digitalWrite(ANALOG_IIC_SCL_PIN,LOW)
#define SCL_Dout_HIGH() digitalWrite(ANALOG_IIC_SCL_PIN,HIGH)
#define SCL_Data_IN() digitalRead(ANALOG_IIC_SCL_PIN)
#define SDA_Write(XX) digitalWrite(ANALOG_IIC_SDA_PIN,(XX?HIGH:LOW))

void Analog_IIC_Init(void);
void Analog_IIC_Start(void);
void Analog_IIC_Stop(void);
void Analog_IIC_Send_Byte(uint8_t txd);
uint8_t Analog_IIC_Read_Byte(uint8_t ack);
void Analog_IIC_NAck(void);
void Analog_IIC_Ack(void);
uint8_t Analog_IIC_Wait_Ack(void);


#endif


