#include    "include.h"
#include    "AllFunction.h"

//蓝牙模块发送的数据数组
int16 OutData[4]={0,0,0,0};
int16 send_b=10000;//10000对应万位，以此类推


/*******************************************************************************
 *  @brief      CRC_CHECK函数
 *  @note       直接放入main中while（1)里执行               
      
                对发送的数据惊醒crc校验：用于虚拟示波器

 *  @warning    18/3/20  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义
 ******************************************************************************/
unsigned short CRC_CHECK(unsigned char *databuf,unsigned char CRC_CNT)
{
  unsigned short CRC_Temp;
  unsigned char k,j;
  CRC_Temp = 0xffff;
  
  for(k=0;k<CRC_CNT;k++)
  {
    CRC_Temp^=databuf[k];
    for(j=0;j<8;j++)
    {if(CRC_Temp&0x01)
      CRC_Temp=(CRC_Temp>>1)^0xa001;
     else CRC_Temp=CRC_Temp>>1;
    }
  }
  return(CRC_Temp);
}

/*
2. * 功能说明： SCI 示波器发送函数，发送一个字节
3. * 参数说明：
4. OutData[] 需要发送的数值赋予该数组
5. * 函数返回：无符号结果值
6. * 修改时间： 2013-2-10
7.  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义*/
 void OutPut_Data(void)
 {
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
 {

  temp[i] = (int)OutData[i];
  temp1[i] = (unsigned int)temp[i];

  }
  for(i=0;i<4;i++)
 {
 databuf[i*2] = (unsigned char)(temp1[i]%256);
 databuf[i*2+1] = (unsigned char)(temp1[i]/256);
 }

 CRC16 = CRC_CHECK(databuf,8);
 databuf[8] = CRC16%256;
 databuf[9] = CRC16/256;

 for(i=0;i<10;i++)
 {
 uart_putchar (UART5,(char)databuf[i]);
 }
 }

/*
2. * 功能说明： SCI 示波器调试函数
3. * 参数说明：
4. OutData[] 需要发送的数值赋予该数组
5. * 函数返回：无符号结果值
6. * 修改时间： 2013-2-10
7.  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义*/
 void OutPut_Data_test(void)
 {
  OutData[0] = 12345;//adc_once(ADC1_SE10, ADC_12bit);
  OutData[1] = 12345;//adc_once(ADC1_SE12, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
  OutData[2] = 12345;//adc_once(ADC1_SE14, ADC_12bit);
  OutData[3] = 12345;//adc_once(ADC1_SE15, ADC_12bit);
  OutPut_Data();
  //printf("ADC_FroBack_6[1]=%d\n",OutData[0]);
 }

/*******************************************************************************
 *  @brief      串口调试助手函数
 *  @note       直接放入main中while（1)里执行               
      
                将需要的数据变成字符串发送回来：用于sscom

 *  @warning   2018.3.20 4.1  移植时，移植时要注意到，send_b，OutData[]，已经在text.c文件定义
 ******************************************************************************/
void OutPut_Data_test_sscom(void)
{
  int16 databuff[20];
  unsigned char l;
  send_b=10000;//给这个数赋值
  OutData[0] = ADC_Value[0];//adc_once(ADC1_SE10, ADC_12bit);
  OutData[1] = ADC_Value[1];//adc_once(ADC1_SE12, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
  OutData[2] = ADC_Value[2];//adc_once(ADC1_SE14, ADC_12bit);
  OutData[3] = ADC_Value[3];//adc_once(ADC1_SE15, ADC_12bit);
  //printf("ADC_FroBack_6[1]=%d\n",OutData[0]);
  for(l=0;l<5;l++)
  {  

   databuff[l*4] = (OutData[0]/send_b);//首先存入高位，依次存入
   databuff[l*4+1] = (OutData[1]/send_b);
   databuff[l*4+2] = (OutData[2]/send_b);
   databuff[l*4+3] = (OutData[3]/send_b);
   OutData[0]=OutData[0]%send_b;//将最高位去掉
   OutData[1]=OutData[1]%send_b;
   OutData[2]=OutData[2]%send_b;
   OutData[3]=OutData[3]%send_b;
   send_b=send_b/10;//除以10，以便下次得到低位
  }
  send_b=10000;//重新给这个数赋值
  
  for(l=0;l<20;l++) //将数据变为对应ASC码
   databuff[l] = databuff[l] + 48;
  
  
 /* databuff[1]=50;
  databuff[2]=50;
  databuff[3]=50;
 */ 
  uart_putchar (UART5,'a');//在串口助手上显示第数据的识别符，例如：发送12345，串口上显示a12345
   for(l=0;l<5;l++)//发送OutData[]第一个数据//此段位发送储存的数据
   {
     uart_putchar (UART5,(char)databuff[l*4]);
    }
   uart_putchar (UART5,'b');
   for(l=0;l<5;l++)//发送OutData[]第二个数据
   {
     uart_putchar (UART5,(char)databuff[l*4+1]);
    }
   uart_putchar (UART5,'c');
   for(l=0;l<5;l++)//发送OutData[]第三个数据
   {
     uart_putchar (UART5,(char)databuff[l*4+2]);
    }
   uart_putchar (UART5,'d');
   for(l=0;l<5;l++)//发送OutData[]第四个数据
   {
     uart_putchar (UART5,(char)databuff[l*4+3]);
    }
}

