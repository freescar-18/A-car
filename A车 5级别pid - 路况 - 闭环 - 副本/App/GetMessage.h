/*!
 * @file       GetMessage.h
 * @brief      数据采集函数
 * @author     
 * @version    A车
 * @date       
 */

#ifndef __GETMESSAGE_H__
#define __GETMESSAGE_H__

/**************************  声明函数体  **************************************/
void GetMessage(void);
void MessageProcessing(void);
void ADCerror_diff(void);
void ADCnormal(void);
void road_check(void);
void Road_Id_Get(void);
void Road_Message(void);
void Round_about(void);
/****************************  宏定义  ****************************************/
#define SamplingNum     (18)  //电感扫描的次数
#define Min_SamplingNum (3)  //滤波时滤去首位的个数
#define DUOJI_RUHUAN    820

#define dataNum     19//储存的归一化后的电感值个数 最好是奇数


#endif  //__GETMESSAGE_H__
    
