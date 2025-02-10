#ifndef __TOFSENSE_F_IIC_H
#define __TOFSENSE_F_IIC_H
#include "analog_iic.h"

#define ADDR_SLAVE 0x08//7位从机地址=ID+0x08
#define ADDR_SLAVE1 0x09//7位从机地址=ID+0x08
#define ADDR_SLAVE2 0x0a//7位从机地址=ID+0x08
#define ADDR_SLAVE3 0x0b//7位从机地址=ID+0x08
#define ADDR_SLAVE4 0x0c//7位从机地址=ID+0x08
//#define ADDR_WRITE_SLAVE (uint8_t)((ADDR_SLAVE<<1)|0x00)//8位从机写地址
//#define ADDR_READ_SLAVE (uint8_t)((ADDR_SLAVE<<1)|0x01)//8位从机读地址

//寄存器变量大小地址列表（常用）
//#define TOF_REGISTER_SIZE 4//单个寄存器长度
#define TOF_REGISTER_TOTAL_SIZE 48//所有寄存器总共长度

#define TOF_ADDR_MODE 0x0c//模式变量地址
#define TOF_SIZE_MODE 1//模式变量所占的字节数

#define TOF_ADDR_ID 0x0d//ID变量地址
#define TOF_SIZE_ID 1//ID变量所占的字节数

#define TOF_ADDR_UART_BAUDRATE 0x10//UART波特率变量地址
#define TOF_SIZE_UART_BAUDRATE 4//UART波特率变量所占的字节数

#define TOF_ADDR_SYSTEM_TIME 0x20//系统时间变量地址
#define TOF_SIZE_SYSTEM_TIME 4//系统时间变量所占的字节数

#define TOF_ADDR_DIS 0x24//距离变量地址
#define TOF_SIZE_DIS 4//距离变量所占的字节数

#define TOF_ADDR_DIS_STATUS 0x28//距离状态指示变量地址
#define TOF_SIZE_DIS_STATUS 2//距离状态指示变量所占的字节数

#define TOF_ADDR_SIGNAL_STRENGTH 0x2a//信号强度变量地址
#define TOF_SIZE_SIGNAL_STRENGTH 2//信号强度变量所占的字节数

#define TOF_ADDR_RANGE_PRECISION 0x2c//测距精度变量地址
#define TOF_SIZE_RANGE_PRECISION 1//测距精度变量所占的字节数

#define IIC_CHANGE_TO_UART_DATA 0x40//将通信模式改为UART需要发送的字节数据

typedef struct {
	uint8_t id;//ID
	uint8_t interface_mode;//通信接口模式，0-UART，1-CAN，2-I/O，3-IIC
	uint32_t uart_baudrate;//UART波特率
	uint32_t system_time;//系统时间
  float dis;//距离
  uint16_t dis_status;//距离状态指示
	uint16_t signal_strength;//信号强度
	uint8_t range_precision;//测距精度
} tofsense_f_output_parameter;//TOFSense-F输出的参数结构体

extern uint8_t iic_read_buff[256];//读取缓存数组
extern uint8_t iic_write_buff[256];//写入缓存数组
extern uint16_t iic_test_count;
extern uint16_t iic_test_i;
extern tofsense_f_output_parameter tofsense_f_output;//解码后存放TOF输出数据的结构体

uint8_t IIC_Unpack_Data(uint8_t scl_pin, uint8_t sda_pin, uint8_t *pdata,uint8_t slave_addr,tofsense_f_output_parameter *pdata1);//通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
uint8_t IIC_Change_Mode_To_UART(uint8_t scl_pin, uint8_t sda_pin, uint8_t slave_addr);//通过IIC将通信模式改为UART模式
uint8_t IIC_Get_All_Register_Data(uint8_t scl_pin, uint8_t sda_pin, uint8_t *pdata,uint8_t slave_addr);//通过IIC按顺序读取所有寄存器的数据并存入指定数组
uint8_t TOF_IIC_Read_One_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t Addr,uint8_t slave_addr);//通过IIC读取指定地址的单个字节数据
uint8_t TOF_IIC_Read_N_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t Addr,uint8_t num,uint8_t *pdata,uint8_t slave_addr);//通过IIC读取指定地址的N个字节数据，存入指定的数组中
uint8_t TOF_IIC_Write_One_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t Addr,uint8_t data,uint8_t slave_addr);//通过IIC写入指定地址单个字节数据
uint8_t TOF_IIC_Write_N_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t Addr,uint8_t num,uint8_t *pdata,uint8_t slave_addr);//通过IIC写入指定地址多个字节数据

uint8_t IIC_Unpack_Dist(uint8_t scl_pin, uint8_t sda_pin, uint8_t Addr, uint8_t num_bytes, uint8_t *pdata,uint8_t slave_addr,tofsense_f_output_parameter *pdata1);
uint8_t IIC_Get_Dist_Data(uint8_t scl_pin, uint8_t sda_pin, uint8_t Addr, uint8_t num_bytes, uint8_t *pdata,uint8_t slave_addr);

#endif
