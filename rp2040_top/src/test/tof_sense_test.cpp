//*******************Nooploop*******************
//TOFSense-F系列Arduino IIC驱动V1.0 2022.08.24
//官方网站：https://www.nooploop.com/
//淘宝商城：https://nooploop.taobao.com/
//深圳空循环科技有限公司
//用精准定位技术为行业赋能！
//*******************Nooploop*******************
//准备工作：需要先将TOFSense-F模块通过USB转TTL模块连接NAssistant配置为IIC模式，级联情况下所有模块的ID不能重复
//接线方式：Arduino板子使用USB等接口供电，所有TOF-F模块的IIC_SDA接到Arduino板子的PD2引脚，所有TOF-F模块的IIC_SCL接到Arduino板子的PD3引脚，TOF-F模块的VCC和GND接到Arduino板子的5V和GND引脚
//注：本例程基于Arduino Nano 168P开发，其余开发板需要先在菜单栏工具中选择开发板和处理器，选择对应端口然后编译下载
//下载程序时需要断开其它模块和Arduino板子之间的TX和RX接线，下载后接好线，点击右上角的串口监视器即可查看到串口输出的TOF-F数据
//*******************Nooploop*******************
#include "analog_iic.h"
#include "tofsense_f_iic.h"
// #include <Wire.h>

tofsense_f_output_parameter tof_f0;//存放ID为0的TOFSense-F输出数据的结构体
tofsense_f_output_parameter tof_f1;//存放ID为1的TOFSense-F输出数据的结构体
unsigned long start_time = 0, cur_time = 0, duration = 0;
uint8_t scl_pin = 1, sda_pin = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);//初始化串口波特率到115200

  // Wire.setSCL(1);
  // Wire.setSDA(0);
  // Wire.begin(); 
  // Wire.setClock(400000); 

  Analog_IIC_Init(scl_pin, sda_pin);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  start_time = millis();

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,ADDR_SLAVE,&tof_f0);//通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
  cur_time = millis();
  duration = cur_time - start_time;
  Serial.print("Read Sensor 1:");
  Serial.println(duration);
  start_time = millis();

  IIC_Unpack_Data(scl_pin, sda_pin, iic_read_buff,ADDR_SLAVE1,&tof_f1);//通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
  cur_time = millis();
  Serial.print("Read Sensor 2:");
  Serial.println(duration);
  start_time = millis();

  //通过串口打印数据
  // Serial.print("id:");
  // Serial.println(tof_f0.id);
  // Serial.print("system_time:");
  // Serial.println(tof_f0.system_time);
  // Serial.print("dis:");
  // Serial.println(tof_f0.dis);
  // Serial.print("dis_status:");
  // Serial.println(tof_f0.dis_status);
  // Serial.print("signal_strength:");
  // Serial.println(tof_f0.signal_strength);
  // Serial.print("range_precision:");
  // Serial.println(tof_f0.range_precision);
  Serial.println("");

  Serial.print("id:");
  Serial.println(tof_f1.id);
  // Serial.print("system_time:");
  // Serial.println(tof_f1.system_time);
  Serial.print("dis:");
  Serial.println(tof_f1.dis);
  Serial.print("dis_status:");
  Serial.println(tof_f1.dis_status);
  // Serial.print("signal_strength:");
  // Serial.println(tof_f1.signal_strength);
  // Serial.print("range_precision:");
  // Serial.println(tof_f1.range_precision);

  cur_time = millis();
  Serial.print("Printed results:");
  Serial.println(duration);
  Serial.println("");
  start_time = millis();
  
  // delay(10);//每100ms查询一次

  
}
