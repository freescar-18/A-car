/*!
 * @file       GetMessage.c
 * @brief      数据采集函数
 * @author     
 * @version    A车
 * @date       
 */
   
/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_power;
uint16 ADC_GetMessage[4][SamplingNum]; //采集回来的电感值，一个电感共 SamplingNum 次
uint16 ADC_Value[4] = {0,0,0,0}; //滤波取平均后的电感值
uint16 SUM_ADC_GetMessage[4] = {0,0,0,0}; 
uint16 ADC_Maxing[4] = {3300,3300,3300,3300}; //电感的最大值（后期需要测）
float ADC_Normal[4] = {0,0,0,0}; //电感归一化后的值（范围 0~1 ）
float fe = 0; //输出的误差
float fe1,fe2;
float fe_last; //上一次的误差
float fec; //误差的变化率
float error_diff_c;
float error_diff;
float error_diff_last;
extern float P_power;
extern float D_power;
uint16 i,j,k;
uint16 jishu=0;
uint16 change; //用于交换参数时所用的中介
uint8 cross_rank = 0,line_rank = 0;
uint8 flag = 0;    //用于停车的标志位 
                  //如果电感值都变小 则 flag 变 1，进入while（flag == 1）死循环中 
                 //按下按键即可将 flag 变 0，跳出死循环重新启动
                 //


float ADC_Yuanhuan_L1=0.000,ADC_Yuanhuan_L2=0.000,ADC_Yuanhuan_L3=0.000,ADC_Yuanhuan_L4=0.000,ADC_Yuanhuan_L5=0.000;

extern int16 steerctrl;
extern int16 last_steerctrl;
extern int16 speed_forecast;

uint8 level;
float dreams = 0.07;
uint16 cross = 0;
uint16 cross_pass = 0;
uint8 none_steerctrl = 0; 
uint16 cross_left = 0;
uint cross_right = 0;

uint8 none_speedctrl = 0; 

/*uint8 n_flag=0,rhd_n_flag=0,rhd_n_flag_a=0,rhd_n_flag_b=0,rhd_n_flag_c=0,rhd_n_flag_d=0,rhd_n_flag_e=0,rhd_n_flag_f=0,rhd_n_flag_g=0;  //逆向入环岛标志
uint8 s_flag=0,rhd_s_flag=0,rhd_s_flag_a=0,rhd_s_flag_b=0,rhd_s_flag_c=0,rhd_s_flag_d=0,rhd_s_flag_e=0,rhd_s_flag_f=0,rhd_s_flag_g=0;  //顺向入环岛标志
uint8 chd_n_flag=0,chd_n_flag_a=0,chd_n_flag_b=0;                                //逆向出环岛标志
uint8 chd_s_flag=0,chd_s_flag_a=0,chd_s_flag_b=0;                                //顺向出环岛标志
int Island_length = 0; 
uint8 shizi_flag=0;
*/
uint8 n_before_is=0,n_before_is_a=0,n_before_is_b=0,n_before_is_c=0;
uint8 n_enter_is=0,n_enter_is_a=0,n_enter_is_b=0;
uint8 n_inside_is=0,n_inside_is_a=0;
uint8 n_exit_is=0,n_exit_is_over=0;
uint8 s_before_is=0,s_before_is_a=0,s_before_is_b=0,s_before_is_c=0;
uint8 s_enter_is=0,s_enter_is_a=0,s_enter_is_b=0;
uint8 s_inside_is=0,s_inside_is_a=0;
uint8 s_exit_is=0,s_exit_is_over=0;
/*******************************************************************************
 *  @brief      MessageProcessing函数
 *  @note       ADC信息采集处理，无归一化 
                结果为每个电感采集SamplingNum次后的平均值（去掉首尾）
 *  @warning    18/3/9 v3.1
 ******************************************************************************/
void MessageProcessing(void)
{  
    for(i = 0;i < SamplingNum; i++)//采集电感SamplingNum次
    {   
        //var_test1 = adc_once(ADC1_SE10, ADC_12bit);
        ADC_GetMessage[0][i] = adc_once(ADC1_SE11, ADC_12bit); //Green
        ADC_GetMessage[1][i] = adc_once(ADC1_SE12, ADC_12bit); //blue
        //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
        ADC_GetMessage[2][i] = adc_once(ADC1_SE14, ADC_12bit); //brown
        ADC_GetMessage[3][i] = adc_once(ADC1_SE15, ADC_12bit);  //orange
    }
    
    for(i = 0;i < (SamplingNum - 1); i++)  //冒泡法排序 从小到大
        for(j = i + 1;j < SamplingNum; j++)
            for(k = 0;k < 4; k++)  //选择电感
            {
                if( ADC_GetMessage[k][i] >= ADC_GetMessage[k][j] )//交换两个数
                {
                    change = ADC_GetMessage[k][i];
                    ADC_GetMessage[k][i] = ADC_GetMessage[k][j];
                    ADC_GetMessage[k][j] = change;
                }
            }
    
    for(i = Min_SamplingNum;i < SamplingNum - Min_SamplingNum; i++)//电感求和
        for(k = 0;k < 4; k++)
        {
            SUM_ADC_GetMessage[k] += ADC_GetMessage[k][i];
        }
    
    for(k = 0;k < 4; k++)//取平均
    {
        ADC_Value[k] = SUM_ADC_GetMessage[k] / (SamplingNum - 2 * Min_SamplingNum);
    }
    
    for(k = 0;k < 4; k++)//清空求和的数组，以便下一次使用
    {
        SUM_ADC_GetMessage[k] = 0;
    }
    
}

/*******************************************************************************
 *  @brief      ADCnormal函数
 *  @note       归一化函数，用每个电感除以电感的最大值
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCnormal(void)
{ 
  for(k = 0;k < 4; k++)
    {
        ADC_Normal[k] = (float)ADC_Value[k] / (float)ADC_Maxing[k];
    }
  if ( ADC_Normal[1] < 0.001) ADC_Normal[1] = 0.001;
  if ( ADC_Normal[2] < 0.001) ADC_Normal[2] = 0.001;
  if ( ADC_Normal[3] < 0.001) ADC_Normal[3] = 0.001;
  if ( ADC_Normal[0] < 0.001) ADC_Normal[0] = 0.001;
}


/*******************************************************************************
 *  @brief      ADCerror_diff函数
 *  @note       偏差法求出误差
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCerror_diff(void)
{
    /*error_diff_last = error_diff;   
    error_diff = ADC_Normal[2] - ADC_Normal[1];
    if(error_diff >= 0)
    {
        fe = error_diff;
    }
    else
    {
        fe = -error_diff;
    }
    error_diff_c = error_diff - error_diff_last;
    if(error_diff_c >= 0)
    {
        fec = error_diff_c;
    }
    else
    {
        fec = -error_diff_c;
    }*/
  
      fe_last = fe;  //记录上一次的值  (ADC_Normal[0] * ADC_Normal[0])
    //  fe = (ADC_Normal[2] - ADC_Normal[1]) * 100;  //直接算两个电感偏差，放大100倍  (ADC_Normal[3] * ADC_Normal[3])
      fe1 =  sqrt(  ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
      fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
      fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
       if( fe < -100) fe = -100;
       if( fe > 100 ) fe = 100;
      fec = fe - fe_last;  //算出变化率
      
   // fe = 0.65 * (ADC_Normal[2] - ADC_Normal[1]) + 0.35 * (ADC_Normal[3] - ADC_Normal[0]);
}


/*******************************************************************************
 *  @brief      road_check函数
 *  @note       获取最大电感值函数
                
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void road_check(void)
{
    D_power = 1;
    if( (cross_pass > 1) || ( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ((ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5)) ) )
    {
        if( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ( (ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5) ))   //判断环岛
        {
            speed_power = 0.1;
            P_power = 2;
            cross = cross + 2;
            if( ADC_Normal[3] > ADC_Normal[0] )  
                cross_left++;
            if( ADC_Normal[0] > ADC_Normal[3] ) 
                cross_right++;
            if( (cross > 250) && (cross_pass < 50) )  
                cross_pass = cross_pass + 100;          
        }
        if( (cross_pass > 1) )
        {
            speed_power = 0.1;
            level = 31;
            if(cross > 0) cross--;
            if(cross_pass > 0) cross_pass--;
            if( cross_left > cross_right )  
            {
                fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe
                fe2 =  sqrt( ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
               // none_steerctrl = 1; steerctrl =  Maxsteering;
            }//左转
            else                           
            {
                //none_steerctrl = 1; steerctrl = Minsteering; 
            }//右转
        }   
    } 
    else 
    {       
            if(cross > 0) cross--;
            if(cross_left > 0) cross_left--;
            if(cross_right > 0) cross_right--;
            none_steerctrl = 0;
            if(ADC_Normal[0] < 0.05 && ADC_Normal[3] < 0.05 )   // 直道 
            {
                if( P_power > 0.5 ) P_power = P_power - 0.8 * dreams;   //缓和减少P值 
                fe1 =  sqrt( 2 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe
                fe2 =  sqrt( 2 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) )  * 100);
                D_power = 0.4;
                speed_power = 1.2;
                level = 1;
            }
            else
            {
               if( ( ADC_Normal[0] < 0.4 || ADC_Normal[3] < 0.4 ) ) //在弯道范围内
               {
                  if( (ADC_Normal[0] > ADC_Normal[3] && ADC_Normal[2] > ADC_Normal[1] && ADC_Normal[0] > 0.05) || ( ADC_Normal[3] > ADC_Normal[0] && ADC_Normal[1] > ADC_Normal[2] && ADC_Normal[3] > 0.05 ) )  //判断是否内切
                  {
                        fe = (int)( (sqrt( ADC_Normal[2] * ADC_Normal[2] ) - sqrt( ADC_Normal[1] * ADC_Normal[1] ) ) * 100 ); // 系数待定
                        level = 11;
                        if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                        speed_power = 0.8;
                  }
                  else
                  {      
                        if( ADC_Normal[0] < 0.05 || ADC_Normal[3] < 0.05 )  //判断是否有一个直的偏小
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2 )  //一个直的偏小，一个直的偏中
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 2; 
                            }             
                            else     //一个直的偏小，一个直的偏大 
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe *0.5会使误差变大
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                        }
                        else 
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2)   //电感都大于小 且有一个偏中
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] +  ADC_Normal[3] * ADC_Normal[3] );  //重新计算fe  *0.5会使误差变大
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] +  ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                            
                            else     //两个电感偏大
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //缓和增加P值
                                speed_power = 0.8;
                                level = 2;
                            }
                        }
                  }
               }
                
            }
            if(ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.4 )   //判断十字
            {
                if( P_power < 1.5 ) P_power = P_power + dreams;   //缓和增加P值
                D_power = 0.6;
                speed_power = 0.5;
                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] );  //重新计算fe
                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                level = 21;
            }

    }
    
    if(fe > 100) fe = 100; //误差保护
    if(fe < -100) fe = -100; //误差保护
}
/*******************************************************************************
 *  @brief      Road_Id_Get函数
 *  @note       偏差法求出误差
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void Road_Id_Get()
{
   /*****  Part 1 丢线判断 *****/
        if( ((ADC_Normal[0] <= 0.500) || (ADC_Normal[1] <= 0.500)) && ((ADC_Normal[0] >= 0.020) || (ADC_Normal[1] >= 0.020)) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //如果右边两个电感均偏大，左边偏小，则角度打死
        {                                                                                                        //  待改
            steerctrl = Minsteering;//last_steerctrl; //舵机输出变最小，向右偏
           // beep_on(); //如果打死则蜂鸣器响一下
           // DELAY_MS(20);
           // beep_off();
        }
        
        if( ((ADC_Normal[2] <= 0.500) || (ADC_Normal[3] <= 0.500)) && ((ADC_Normal[2] >= 0.020) || (ADC_Normal[3] >= 0.020)) && (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) )//如果左边两个电感均偏大，右边偏小，则角度打死
        {                                                                                                        //  待改
             
            steerctrl = Maxsteering;//last_steerctrl; //舵机输出变最大，向左偏
          //  beep_on(); //如果打死则蜂鸣器响一下
          //  DELAY_MS(20);
          //  beep_off();
        }
                if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //如果四个电感都偏小，并且是初始状态，则将flag变成1，然后进入下面的死循环
        {                                                                                                 
          jishu++;
          steerctrl = last_steerctrl;
        }
   
         if(jishu >= 10) //几十毫秒后，还是那么小，flag=1，下面进入停车
        {
          flag = 1;
         //speed_forecast = 0;                  
        }
        
        if(( (ADC_Normal[0] >= 0.005) || (ADC_Normal[1] >= 0.005) || (ADC_Normal[2] >= 0.005) || (ADC_Normal[3] >= 0.005))&&(jishu < 10) ) //电感值恢复，则将jishu清空，继续正常跑
        {                                                                                                 
          jishu = 0;
        }        
          
        /*****  Part 2  路况识别*****/
        
///////////////////////////////////////////环岛逆向入环////////////////////////////////
       

          
     
        
        
        /*****  Part 3 停车判断  *****/
       /* if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
        {                                                                                                 
          flag = 1;                                                                                       
        }*/     
        if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
        {
         speed_forecast = 0;
         
         //steerctrl = 768;
         
           // ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //电机输出 0
            //ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //电机输出 0
       //     LED_PrintShort(50,7,speed_forecast); //显示电机PWM
        //  LED_PrintValueF(50,5,speed_fec_max,2); //显示最大误差变化率的绝对值
        //     LED_PrintValueF(50,3,speed_min,2); //显示最小速度
        //     DELAY_MS(100);
             
        }
}

/*******************************************************************************
 *  @brief      Round_about函数
 *  @note       出入环岛区域
                
 *  @warning    
 ******************************************************************************/
void Round_about() 
{
  //环岛逆向/////
  if((n_before_is == 0)&&(n_enter_is == 0)&&(s_inside_is == 0)&&(n_exit_is == 0)&&(n_exit_is_over == 0)&&(ADC_Normal[1] >= 0.500)&&(ADC_Normal[2] >= 0.900))
  {
    n_before_is = 1;   //入岛前
  }
  if(n_before_is == 1)   //入岛前区域  beep
  {
     speed_power = 0.5;
     //////////寻找第一个波峰
     if(( n_before_is_a == 0)&&( n_before_is_b == 0)&&(ADC_Normal[3] >= 0.200))
     {
       n_before_is_a = 1;
       beep_on();
     }
     //////////寻找第一个波谷
     if((n_before_is_a == 1)&&(n_before_is_b == 0)&&(ADC_Normal[3] <= 0.100))
     {
       n_before_is_a = 0;
       n_before_is_b = 1;
       beep_off();
     }
     //////////寻找第二个波谷
     if((n_before_is_b == 1)&&(ADC_Normal[3] >= 0.100))
     {
       n_before_is_b = 0;
       n_before_is = 0;
       n_enter_is = 1;      //入岛点
     }
  }
  if(n_enter_is == 1)     //入岛过程   beep
  {
     speed_power = 0.1;
     beep_on();
     //////////策略一：换误差算法///////// 
     fe_last = fe;                //入岛误差算法
     fe = (int)(( (sqrt(ADC_Normal[2]) - sqrt(ADC_Normal[3])) / ( ADC_Normal[2] + ADC_Normal[3] ) ) * 100);
     fec = fe - fe_last;
     //////////避免打角太小//////////
     if(steerctrl >= Midsteering + 120)  
        n_enter_is_a = 1;
     if((n_enter_is_a == 1)&&(steerctrl <= Midsteering + 60))
     {     none_steerctrl = 1;
           steerctrl = Maxsteering;
     }
    
   /*  //////////策略二：打死角/////////////
     none_steerctrl = 1;
     steerctrl = Maxsteering ;
   */  
     //////////避免提早结束入岛///////  
     if(ADC_Normal[1] >= 0.800)
        n_enter_is_b = 1;
     ///////////入岛结束点////////////
     if((n_enter_is_b == 1)&&((ADC_Normal[1] < 0.600)||(ADC_Normal[2] < 0.600)))
     {
       speed_power = 1.0;
       beep_off();
       none_steerctrl = 0;
       
       n_enter_is_a = 0;
       n_enter_is_b = 0;
       n_enter_is = 0;
       
       n_inside_is = 1;       
     }
              
  }
  if(n_inside_is == 1)  //岛内过程
  {
    speed_power = 1.0;
    none_steerctrl = 0;
    //beep_on();
    if((n_inside_is_a == 0)&&(ADC_Normal[3] <= 0.300))
    {
      n_inside_is_a = 1;
    }
    if((n_inside_is_a == 1)&&(ADC_Normal[3] >= 0.700))
    {
      n_inside_is_a = 0;
      n_inside_is = 0;
      n_exit_is = 1; 
      //beep_off();
    }
  }
  if(n_exit_is == 1)       //出岛过程   beep
  {
     speed_power = 0.5;
     beep_on();
     ////////出岛误差算法//////////  
     fe_last = fe;                    
     fe = (int)( (ADC_Normal[3] - ADC_Normal[0] ) * 100);
     fec = fe - fe_last;
     ////////出岛结束点////////////
     if(ADC_Normal[3] <= 0.700)
     {
       speed_power = 1.0;
       beep_off();
       n_exit_is = 0;
       n_exit_is_over = 1;  //
     }
  }
  if(n_exit_is_over == 1)   //离开环岛区域
  {
    speed_power = 1.0;
    none_steerctrl = 0;
    if((ADC_Normal[0] <= 0.050)&&(ADC_Normal[3] <= 0.050))
      n_exit_is_over = 0;
  }
    
    //环岛顺向/////
  if((s_before_is == 0)&&(s_enter_is == 0)&&(s_inside_is == 0)&&(s_exit_is == 0)&&(s_exit_is_over == 0)&&(ADC_Normal[2] >= 0.500)&&(ADC_Normal[1] >= 0.900))
  {
    s_before_is = 1;   //入岛前
  }
  if(s_before_is == 1)   //入岛前区域  beep
  {
     speed_power = 0.5;
     //////////寻找第一个波峰
     if(( s_before_is_a == 0)&&( s_before_is_b == 0)&&(ADC_Normal[0] >= 0.200))
     {
       s_before_is_a = 1;
       beep_on();
     }
     //////////寻找第一个波谷
     if((s_before_is_a == 1)&&(s_before_is_b == 0)&&(ADC_Normal[0] <= 0.100))
     {
       s_before_is_a = 0;
       s_before_is_b = 1;
       beep_off();
     }
     //////////寻找第二个波谷
     if((s_before_is_b == 1)&&(ADC_Normal[0] >= 0.100))
     {
       s_before_is_b = 0;
       s_before_is = 0;
       s_enter_is = 1;      //入岛点
     }
  }
  if(s_enter_is == 1)     //入岛过程  beep
  {
     speed_power = 0.1;
     beep_on();
   /*  //////////策略一：换误差算法///////// 
     fe_last = fe;                //入岛误差算法
     fe = (int)(( (sqrt(ADC_Normal[1]) - sqrt(ADC_Normal[0])) / ( ADC_Normal[1] + ADC_Normal[0] ) ) * 100);
     fec = fe - fe_last;
     //////////避免打角太小//////////
     if(steerctrl <= Midsteering - 120)  
        s_enter_is_a = 1;
     if((s_enter_is_a == 1)&&(steerctrl >= Midsteering - 60))
     {     none_steerctrl = 1;
           steerctrl = Minsteering;
     }
    */
     //////////策略二：打死角/////////////
     none_steerctrl = 1;
     steerctrl = Minsteering ;
     
     //////////避免提早结束入岛///////  
     if(ADC_Normal[2] >= 0.800)
        s_enter_is_b = 1;
     ///////////入岛结束点////////////
     if((s_enter_is_b == 1)&&((ADC_Normal[1] < 0.600)||(ADC_Normal[2] < 0.600)))
     {
       speed_power = 1.0;
       beep_off();
       none_steerctrl = 0;
       
       s_enter_is_a = 0;
       s_enter_is_b = 0;
       s_enter_is = 0;      
       s_inside_is = 1;       
     }
              
  }
  if(s_inside_is == 1)  //岛内过程
  {
    speed_power = 1.0;
    none_steerctrl = 0;
    if((n_inside_is_a == 0)&&(ADC_Normal[0] <= 0.300))
      n_inside_is_a = 1;
    if((n_inside_is_a == 1)&&(ADC_Normal[0] >= 0.700)&&((ADC_Normal[1]-ADC_Normal[0])*(ADC_Normal[1]-ADC_Normal[0]) <= 0.250))
    {
      n_inside_is_a = 0;
      n_inside_is = 0;
      s_exit_is = 1;
    }
  }
  if(s_exit_is == 1)       //出岛过程  beep
  {
     speed_power = 0.5;
     beep_on();
     ////////出岛误差算法//////////  
     fe_last = fe;                    
     fe = (int)( (ADC_Normal[0] - ADC_Normal[3] ) * 100);
     fec = fe - fe_last;
     ////////出岛结束点////////////
     if(ADC_Normal[0] <= 0.700)
     {
       speed_power = 1.0;
       beep_off();
       s_exit_is = 0;
       s_exit_is_over = 1;  //
     }
  }
  if(s_exit_is_over == 1)
  {
    speed_power = 1.0;
    none_steerctrl = 0;
    if((ADC_Normal[0] <= 0.050)&&(ADC_Normal[3] <= 0.050))
      s_exit_is_over = 0;
  }
 
}





