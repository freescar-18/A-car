/*!
 * @file       WIRELESS.h
 * @brief      各类函数
 * @author     
 * @version    B车
 * @date       
 */

#ifndef __WIRELESS__
#define __WIRELESS__

/**************************  声明函数体  **************************************/
unsigned short CRC_CHECK(unsigned char *databuf,unsigned char CRC_CNT);
void OutPut_Data_test(void);
extern void OutPut_Data_test_sscom(void);
void nrf_exchange_data(void);

void USendOneByte(UARTn_e uratn, uint8 ch);
void push(uint8 chanel,uint16 data);
void sendDataToScope(void);
void test_scope(void);
/****************************  宏定义  ****************************************/

#endif  //__WIRELESS__  