//*******************Nooploop*******************
//TOFSense/TOFSense-F系列Arduino驱动V1.0 2022.08.20
//官方网站：https://www.nooploop.com/
//淘宝商城：https://nooploop.taobao.com/
//深圳空循环科技有限公司
//用精准定位技术为行业赋能！
//*******************Nooploop*******************
//准备工作：需要先将TOF模块通过USB转TTL模块连接NAssistant配置为UART主动输出模式，波特率设置为115200
//接线方式：Arduino板子使用USB等接口供电，TOF模块的TX接到Arduino板子的串口RX引脚，TOF模块的RX接到Arduino板子的串口TX引脚，TOF模块的VCC和GND接到Arduino板子的5V和GND引脚
//注：本例程基于Arduino Nano 168P开发，其余开发板需要先在菜单栏工具中选择开发板和处理器，选择对应端口然后编译下载
//下载程序时需要断开TOF和Arduino板子之间的TX和RX接线，下载后接好线，点击右上角的串口监视器即可查看到串口输出的TOF数据
//*******************Nooploop*******************
#include <Arduino.h>
#define TOF_FRAME_HEADER 0x57//定义TOFSense系列和TOFSense-F系列的帧头
#define TOF_FUNCTION_MARK 0x00//定义TOFSense系列和TOFSense-F系列的功能码

#define TX_PIN 12
#define RX_PIN 13

typedef struct {
  unsigned char id;//TOF模块的id
  unsigned long system_time;//TOF模块上电后经过的时间，单位：ms
  float dis;//TOF模块输出的距离，单位：m
  unsigned char dis_status;//TOF模块输出的距离状态指示
  unsigned int signal_strength;//TOF模块输出的信号强度
  unsigned char range_precision;//TOF模块输出的重复测距精度参考值，TOFSense-F系列有效，单位：cm
} tof_parameter;//解码后的TOF数据结构体

unsigned int count_i,count_j=0;//循环计数变量
tof_parameter tof0;//定义一个存放解码后数据的结构体
unsigned char check_sum=0;//校验和
unsigned char rx_buf[32];//串口接收数组

void setup() {
  Serial.begin(115200);//初始化串口波特率到115200
  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  Serial1.begin(921600);
}

void loop() {
  // Serial.println("running");
  if(Serial1.available()>0)//如果串口缓存区接收到了数据
  {
    if(Serial1.peek() == TOF_FRAME_HEADER)//如果串口缓存区接收到的数据是TOF_FRAME_HEADER，说明可能是TOF的数据帧头（peek不清除串口接收缓存区的该数据）
    {
      count_i=0;//数组下标计数变量置0
      rx_buf[count_i]=Serial1.read();//将帧头放入数组第一个元素位置并清除串口接收缓存区的该数据
    }
    else
    {
      rx_buf[count_i]=Serial1.read();//如果不是帧头则正常读取并清除串口接收缓存区的该数据
    }
        
    count_i++;//数组下标计数变量+1，准备将接收到的数据存入下一个位置
    
    if(count_i>15)//如果数组下标计数变量>15，说明接收数组中存满了16个数据，计数变量清零并进行一次解码
    {
      count_i=0;

      for(count_j=0;count_j<15;count_j++)
      {
        check_sum+=rx_buf[count_j];//计算数据的校验和
      }

      if((rx_buf[0] == TOF_FRAME_HEADER)&&(rx_buf[1] == TOF_FUNCTION_MARK)&&(check_sum == rx_buf[15]))//如果接收数组第一和第二个元素分别等于TOF_FRAME_HEADER和TOF_FUNCTION_MARK，且算出的校验和的低字节等于协议中的校验和，说明解码正确
      {
        tof0.id=rx_buf[3];//取TOF模块的id
        tof0.system_time=(unsigned long)(((unsigned long)rx_buf[7])<<24|((unsigned long)rx_buf[6])<<16|((unsigned long)rx_buf[5])<<8|(unsigned long)rx_buf[4]);//取TOF模块上电后经过的时间        
        tof0.dis=((float)(((long)(((unsigned long)rx_buf[10]<<24)|((unsigned long)rx_buf[9]<<16)|((unsigned long)rx_buf[8]<<8)))/256))/1000.0;//取TOF模块输出的距离
        tof0.dis_status=rx_buf[11];//取TOF模块输出的距离状态指示
        tof0.signal_strength=(unsigned int)(((unsigned int)rx_buf[13]<<8)|(unsigned int)rx_buf[12]);//取TOF模块输出的信号强度
        tof0.range_precision=rx_buf[14];//取TOF模块输出的重复测距精度参考值

        //通过串口打印数据
        Serial.print("id:");
        Serial.println(tof0.id);
        Serial.print("system_time:");
        Serial.println(tof0.system_time);
        Serial.print("dis:");
        Serial.println(tof0.dis);
        Serial.print("dis_status:");
        Serial.println(tof0.dis_status);
        Serial.print("signal_strength:");
        Serial.println(tof0.signal_strength);
        Serial.print("range_precision:");
        Serial.println(tof0.range_precision);
        Serial.println("");
      }
      
    }
    check_sum=0;//清空校验和
  }  
}
