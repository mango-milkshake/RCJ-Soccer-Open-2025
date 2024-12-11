#include "analog_iic.h"


/************************************************
函数名称 ： SDA_Output
函数功能 ： 将SDA引脚配置为输出模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SDA_Output(void)
{
	pinMode(ANALOG_IIC_SDA_PIN, OUTPUT);
}

/************************************************
函数名称 ： SDA_Input
函数功能 ： 将SDA引脚配置为输入模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SDA_Input(void)
{
	pinMode(ANALOG_IIC_SDA_PIN, INPUT);
}

/************************************************
函数名称 ： SCL_Output
函数功能 ： 将SCL引脚配置为输出模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SCL_Output(void)
{
	pinMode(ANALOG_IIC_SCL_PIN, OUTPUT);
}

/************************************************
函数名称 ： SCL_Input
函数功能 ： 将SCL引脚配置为输入模式
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SCL_Input(void)
{
	pinMode(ANALOG_IIC_SCL_PIN, INPUT);
}

/************************************************
函数名称 ： Analog_IIC_Init
函数功能 ： 模拟IIC初始化函数
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Init(void)
{
	SCL_Output();
	SDA_Output();
	SCL_Dout_HIGH();
	SDA_Dout_HIGH();
}

/************************************************
函数名称 ： Analog_IIC_Start
函数功能 ： 产生IIC起始信号
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Start(void)
{
	SDA_Output();
	SDA_Dout_HIGH();
	SCL_Dout_HIGH();
	Delay_us(4);
	SDA_Dout_LOW();
	Delay_us(4);
	SCL_Dout_LOW();
}

/************************************************
函数名称 ： Analog_IIC_Stop
函数功能 ： 产生IIC停止信号
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Stop(void)
{
	SDA_Output();
	SCL_Dout_LOW();
	SDA_Dout_LOW();
	Delay_us(4);
	SCL_Dout_HIGH();
	SDA_Dout_HIGH();
	Delay_us(4);
}

/************************************************
函数名称 ： Analog_IIC_Wait_Ack
函数功能 ： 等待从机Ack信号
参    数 ： 无
返 回 值 ： 0 NAck 1 Ack
*************************************************/
uint8_t Analog_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;

	SDA_Input();
	SDA_Dout_HIGH();
	Delay_us(1);
	SCL_Dout_HIGH();
	Delay_us(1);
	while(SDA_Data_IN())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Analog_IIC_Stop();
			return 1;
		}
	}
	SCL_Dout_LOW();//时钟输出0
	return 0;
}

/************************************************
函数名称 ： Analog_IIC_Ack
函数功能 ： 产生Ack应答
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_Ack(void)
{
	SCL_Dout_LOW();
	SDA_Output();
	SDA_Dout_LOW();
	Delay_us(2);
	SCL_Dout_HIGH();
	Delay_us(2);
	SCL_Dout_LOW();
}

/************************************************
函数名称 ： Analog_IIC_NAck
函数功能 ： 不产生Ack应答
参    数 ： 无
返 回 值 ： 无
*************************************************/
void Analog_IIC_NAck(void)
{
	SCL_Dout_LOW();
	SDA_Output();
	SDA_Dout_HIGH();
	Delay_us(2);
	SCL_Dout_HIGH();
	Delay_us(2);
	SCL_Dout_LOW();
}

/************************************************
函数名称 ： Analog_IIC_Send_Byte
函数功能 ： IIC发送一个字节
参    数 ： txd 需要发送的字节
返 回 值 ： 无
*************************************************/
void Analog_IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	//拉低时钟开始数据传输
	SDA_Output();
	SCL_Dout_LOW();
	for(t=0;t<8;t++)
	{
		SDA_Write((txd&0x80)>>7);
		txd<<=1;
		Delay_us(5);
		SCL_Dout_HIGH();
		Delay_us(5);
		SCL_Dout_LOW();
		//Delay_us(2);
  }
}

/************************************************
函数名称 ： Analog_IIC_Read_Byte
函数功能 ： IIC读取一个字节
参    数 ： ack 读取后是否需要发送Ack信号，ack=1时，发送Ack，ack=0，发送NAck
返 回 值 ： receive 读取到的字节
*************************************************/
uint8_t Analog_IIC_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	//SDA设置为输入
	SDA_Input();
  for(i=0;i<8;i++ )
	{
		SCL_Dout_LOW();
		Delay_us(5);
		SCL_Dout_HIGH();
		receive<<=1;
		if(SDA_Data_IN())receive++;
		Delay_us(5);
  }
  if(!ack)
  {
  	Analog_IIC_NAck();//发送nACK
  }
  else
  {
  	Analog_IIC_Ack(); //发送ACK
  }

	return receive;
}






