#include "analog_iic.h"


/************************************************
函数名称 ： SDA_Output
函数功能 ： 将SDA引脚配置为输出模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SDA_Output(uint8_t sda_pin)
{
	pinMode(sda_pin, OUTPUT);
}

/************************************************
函数名称 ： SDA_Input
函数功能 ： 将SDA引脚配置为输入模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SDA_Input(uint8_t sda_pin)
{
	pinMode(sda_pin, INPUT);
}

/************************************************
函数名称 ： SCL_Output
函数功能 ： 将SCL引脚配置为输出模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SCL_Output(uint8_t scl_pin)
{
	pinMode(scl_pin, OUTPUT);
}

/************************************************
函数名称 ： SCL_Input
函数功能 ： 将SCL引脚配置为输入模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SCL_Input(uint8_t scl_pin)
{
	pinMode(scl_pin, INPUT);
}

/************************************************
函数名称 ： Analog_IIC_Init
函数功能 ： 模拟IIC初始化函数
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Init(uint8_t scl_pin, uint8_t sda_pin)
{
	SCL_Output(scl_pin);
	SDA_Output(sda_pin);
	SCL_Dout_HIGH(scl_pin);
	SDA_Dout_HIGH(sda_pin);
}

/************************************************
函数名称 ： Analog_IIC_Start
函数功能 ： 产生IIC起始信号
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Start(uint8_t scl_pin, uint8_t sda_pin)
{
	SDA_Output(sda_pin);
	SDA_Dout_HIGH(sda_pin);
	SCL_Dout_HIGH(scl_pin);
	Delay_us(4);
	SDA_Dout_LOW(sda_pin);
	Delay_us(4);
	SCL_Dout_LOW(scl_pin);
}

/************************************************
函数名称 ： Analog_IIC_Stop
函数功能 ： 产生IIC停止信号
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Stop(uint8_t scl_pin, uint8_t sda_pin)
{
	SDA_Output(sda_pin);
	SCL_Dout_LOW(scl_pin);
	SDA_Dout_LOW(sda_pin);
	Delay_us(4);
	SCL_Dout_HIGH(scl_pin);
	SDA_Dout_HIGH(sda_pin);
	Delay_us(4);
}

/************************************************
函数名称 ： Analog_IIC_Wait_Ack
函数功能 ： 等待从机Ack信号
参    数 ： 无
返 回 值 ： 0 NAck 1 Ack
*************************************************/
uint8_t Analog_IIC_Wait_Ack(uint8_t scl_pin, uint8_t sda_pin)
{
	uint8_t ucErrTime=0;

	SDA_Input(sda_pin);
	SDA_Dout_HIGH(sda_pin);
	Delay_us(1);
	SCL_Dout_HIGH(scl_pin);
	Delay_us(1);
	while(SDA_Data_IN(sda_pin))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Analog_IIC_Stop(scl_pin, sda_pin);
			return 1;
		}
	}
	SCL_Dout_LOW(scl_pin);//时钟输出0
	return 0;
}

/************************************************
函数名称 ： Analog_IIC_Ack
函数功能 ： 产生Ack应答
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Ack(uint8_t scl_pin, uint8_t sda_pin)
{
	SCL_Dout_LOW(scl_pin);
	SDA_Output(sda_pin);
	SDA_Dout_LOW(sda_pin);
	Delay_us(2);
	SCL_Dout_HIGH(scl_pin);
	Delay_us(2);
	SCL_Dout_LOW(scl_pin);
}

/************************************************
函数名称 ： Analog_IIC_NAck
函数功能 ： 不产生Ack应答
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_NAck(uint8_t scl_pin, uint8_t sda_pin)
{
	SCL_Dout_LOW(scl_pin);
	SDA_Output(sda_pin);
	SDA_Dout_HIGH(sda_pin);
	Delay_us(2);
	SCL_Dout_HIGH(scl_pin);
	Delay_us(2);
	SCL_Dout_LOW(scl_pin);
}

/************************************************
函数名称 ： Analog_IIC_Send_Byte
函数功能 ： IIC发送一个字节
参    数 ： txd 需要发送的字节
返 回 值 ： 无
*************************************************/
void Analog_IIC_Send_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t txd)
{
	uint8_t t;
	//拉低时钟开始数据传输
	SDA_Output(sda_pin);
	SCL_Dout_LOW(scl_pin);
	for(t=0;t<8;t++)
	{
		SDA_Write(sda_pin, (txd&0x80)>>7);
		txd<<=1;
		Delay_us(5);
		SCL_Dout_HIGH(scl_pin);
		Delay_us(5);
		SCL_Dout_LOW(scl_pin);
		//Delay_us(2);
  }
}

/************************************************
函数名称 ： Analog_IIC_Read_Byte
函数功能 ： IIC读取一个字节
参    数 ： ack 读取后是否需要发送Ack信号，ack=1时，发送Ack，ack=0，发送NAck
返 回 值 ： receive 读取到的字节
*************************************************/
uint8_t Analog_IIC_Read_Byte(uint8_t scl_pin, uint8_t sda_pin, uint8_t ack)
{
	unsigned char i,receive=0;
	//SDA设置为输入
	SDA_Input(sda_pin);
  for(i=0;i<8;i++ )
	{
		SCL_Dout_LOW(scl_pin);
		Delay_us(5);
		SCL_Dout_HIGH(scl_pin);
		receive<<=1;
		if(SDA_Data_IN(sda_pin))receive++;
		Delay_us(5);
  }
  if(!ack)
  {
  	Analog_IIC_NAck(scl_pin, sda_pin);//发送nACK
  }
  else
  {
  	Analog_IIC_Ack(scl_pin, sda_pin); //发送ACK
  }

	return receive;
}






