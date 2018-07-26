/*!
 * @file       start&stop.c
 * @brief      存放各类之前测试的函数
 * @author     
 * @version    A车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
extern float ADC_Normal[5];
extern uint16 ADC_Value[5];
extern float fe,fec;
extern int16 steerctrl;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
extern int16 speed_now_left,speed_now_right;
extern float speed_power;
extern uint8 none_steerctrl; 
extern int16 speedctrl_left,speedctrl_right; 
extern int16 speedctrl_left_opp,speedctrl_right_opp; 
extern uint16 last_stop;  //终点停车标记位  0为不是停车
extern float speed_forecast; //预测将要达到的速度（PWM）
extern float speed_forecast_error; //预测将要达到的速度的偏差（差速）
extern float fe,fe1,fe2,fe_last;
uint16 start_flag = 0;//开车标记位，1个单位为5ms
                      //例如start_flag = 300，意义为1.5s进入发车程序（ start_car(void) )
int16 dis_left = 0;//左轮走的总距离
int16 dis_right = 0;//右轮走的总距离
extern float P_power;
extern float D_power;
uint16 dis_back = 3000;//倒车后退的距离，会有惯性滑行一下
uint8 left_flag = 0;
uint8 right_flag = 0;
uint8 shizi_turn_flag = 0;
extern uint8 level;
extern uint8 turn_left_flag;
extern uint8 turn_right_flag;
float max_shizi = 0.7;
/*******************************************************************************
 *  @brief      start_car(void) 
 *  @note       发车函数 车会自动向右边跑
 *  @warning    参数定义：start_flag为启动的周期数
                          例如：start_flag = 300,则 300*5 = 1.5s 向右走
                速度：2000
 ******************************************************************************/
void start_car(void)
{           
            start_flag--;
            speed_now_right = ftm_quad_get(FTM2);  //right轮
            speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
            lptmr_pulse_clean();
      
            /*****  Part 1 信息采集 舵机速度PID *****/
            MessageProcessing(); //信息采集
            ADCnormal(); //采集的信息归一化
        
            
 /**/       fe_last = fe;  //记录上一次的值  (ADC_Normal[0] * ADC_Normal[0])
 /**/       fe1 =  sqrt(  ADC_Normal[3] * ADC_Normal[3] );
 /**/       fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
 /**/       fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
 /**/       fec = fe - fe_last;  //算出变化率      
 
         // road_check();
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
            speed_fuzzy_mem_cal_forecast(); //对输入的 fe（误差）的绝对值 和 fec（误差变化率）的绝对值 查询隶属度
            speed_fuzzy_query_forecast(); //查询模糊规则表（速度）
            speed_fuzzy_solve_forecast(); //解模糊 输出 speed_forecast 为预期的速度
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get();
            
  /**/    //  if(ADC_Normal[2] < 0.3  && ADC_Normal[1] < 0.05)  steerctrl = Maxsteering; //左电感太小，向左打角
  /**/    //  else if(ADC_Normal[2] > 0.1 && ADC_Normal[1] > 0.6)  steerctrl = Minsteering; //右电感太大，向右打角
  /**/      if( ADC_Normal[2] < 0.4 &&  ADC_Normal[1] < 0.2 )  steerctrl = Maxsteering;
  /**/      else steerctrl = Minsteering + 15;
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////
            
            ///////////////////////////左轮速度/////////////////////////////////////
            ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,3000); //输出电机PWM  正转
            ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM 
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
            ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,3000); //输出电机PWM  正转
            ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM 
            ///////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
           //   flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
               // ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
               // ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
               // ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
               // ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
}

/*******************************************************************************
 *  @brief      stop_car(void) 
 *  @note       停车
 *  @warning    刹车专用
                标志位：last_flag  使用后记得清last_flag
                100个last_flag之后会自动 flag = 1 ，就是PWM为0  
 ******************************************************************************/
void stop_car(void)
{
            last_stop++;   
            speed_now_right = ftm_quad_get(FTM2);  //right轮
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
	    if(speed_now_right < 0) speed_now_right = -speed_now_right;//右轮不算负的，用于快速反转
            P_power = 0.1;
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();
	  
	    /*****  Part 1 信息采集 舵机速度PID *****/
	    MessageProcessing(); //信息采集
    	    ADCnormal(); //采集的信息归一化
	    ADCerror_diff(); //偏差法计算 误差 和 误差的变化率
            // road_check(); 
		
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
				
            speed_forecast = 0;  //预测速度为0
            speed_forecast_error = 0;  //预测速度为0
            
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get();
            
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////左轮速度/////////////////////////////////////
            if(speedctrl_left < 0)  //左轮速度溢出
            {
                if(speedctrl_left < -9500) speedctrl_left_opp = 9500;
                else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,9500); //输出电机PWM  
            }
            else  //左轮速度没溢出  
            {
                if(speedctrl_left > 100) speedctrl_left = 100;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,9500); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
            if(speedctrl_right < 0)  //右轮速度溢出
            {
                if(speedctrl_right < -9500) speedctrl_right_opp = 9500;
                else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,9500); //输出电机PWM  
            }
            else  //右轮速度没溢出
            {
                if(speedctrl_right > 100) speedctrl_right = 100;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,9500); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
            //    flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
            //if( last_stop < 600 ) last_stop++;  //停车计数
/**/        //if( last_stop >= 100 ) flag = 1; 
/**/        P_power = 1;

            
}  

/*******************************************************************************
 *  @brief      turn_car(void)  
 *  @note       倒车
*  @warning     倒车算距离 使用前定义dis_back，dis_back即为倒车的距离
                           使用后清dis_right
                flag会置1
                速度：1300
 ******************************************************************************/
void turn_car(void)
{
            speed_now_right = ftm_quad_get(FTM2);  //right轮
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();
	  
	    /*****  Part 1 信息采集 舵机速度PID *****/
	    MessageProcessing(); //信息采集
    	    ADCnormal(); //采集的信息归一化
/**/        fe_last = fe;  //记录上一次的值  (ADC_Normal[0] * ADC_Normal[0])
/**/        fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2]);
/**/        fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1]);
/**/        fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);      

/**/        fe = -fe;

/**/        fec = fe - fe_last;  //算出变化率
            // road_check(); 

            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角

		
            ////////////////////////////////////
            dis_right += speed_now_right;
            if( dis_right < - dis_back ) //倒车后退的距离
            {
                flag = 1;
            }
            ////////////////////////////////////
            
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get();
            
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////左轮速度/////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,1600); //输出电机PWM  反转
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1600); //输出电机PWM  反转
            ////////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
            //    flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
           
}   

/*******************************************************************************
 *  @brief      turn_car_shizi(void)  
 *  @note       倒车
*  @warning    从十字倒车回来，然后重新发车，校赛专用，直接使用
                level 会变成 1 
                速度：2000
 ******************************************************************************/
void turn_car_shizi(void)
{
            speed_now_right = ftm_quad_get(FTM2);  //right轮
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();
	  
	    /*****  Part 1 信息采集 舵机速度PID *****/
	    MessageProcessing(); //信息采集
    	    ADCnormal(); //采集的信息归一化
/**/        fe_last = fe;  //记录上一次的值  (ADC_Normal[0] * ADC_Normal[0])
/**/        fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
/**/        fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
/**/        fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);      
/**/        fe = -fe;
/**/        fec = fe - fe_last;  //算出变化率
            // road_check(); 
		
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
            
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get();
  /**/      if((ADC_Normal[0] > 0.6 && ADC_Normal[3] > 0.6))
  /**/      {
  /**/         level = 1; 
  /**/      }
            else
            {
  /**/          if(turn_right_flag == 0)
  /**/              steerctrl = Minsteering;   
  /**/          if(turn_left_flag == 0)
 /**/               steerctrl = Maxsteering;  
            }
            
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////左轮速度/////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,3500); //输出电机PWM  反转
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,3500); //输出电机PWM  反转
            ////////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
            //    flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
           
}   


/*******************************************************************************
 *  @brief      right_car(void) 
 *  @note       向右转
 *  @warning    用于十字倒完车之后向右转
                flag会置1
                使用后清left_flag
                速度：1500
 ******************************************************************************/
void right_car(void)
{           
            speed_now_right = ftm_quad_get(FTM2);  //right轮
            speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
            lptmr_pulse_clean();
      
            /*****  Part 1 信息采集 舵机速度PID *****/
            MessageProcessing(); //信息采集
            ADCnormal(); //采集的信息归一化
	    ADCerror_diff(); //偏差法计算 误差 和 误差的变化率
            
 
         // road_check();
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
            speed_fuzzy_mem_cal_forecast(); //对输入的 fe（误差）的绝对值 和 fec（误差变化率）的绝对值 查询隶属度
            speed_fuzzy_query_forecast(); //查询模糊规则表（速度）
            speed_fuzzy_solve_forecast(); //解模糊 输出 speed_forecast 为预期的速度
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get(); 
            
/**/   //        flag = 0;
/**/    //        dis_right += speed_now_right;
/**/    //        if( dis_right > dis_back ) //倒车后退的距离
/**/    //        {
/**/    //              beep_off();
/**/    //              flag = 1;
/**/    //         }
        //    if(ADC_Normal[0] < 0.1 && ADC_Normal[3] < 0.1 && left_flag == 1)
        //    {
        //        left_flag = 0;
        //    }
  /**/  //    if((ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.6) || left_flag == 1)
  /**/   //  {
  /**/   //       steerctrl = Minsteering;
  /**/   //       left_flag = 1;
  /**/   //   }    440788918087
  /**/      if(  ( dis_right > 1100 ) && right_flag == 1 )
  /**/      {
  /**/          flag = 1;
  /**/      }
  /**/      if((ADC_Normal[0] > max_shizi && ADC_Normal[3] > max_shizi) || right_flag == 1)
  /**/      {
  /**/          steerctrl = Minsteering + 20;
  /**/          right_flag = 1;
                dis_right += speed_now_right;
  /**/      }
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////
            
            ///////////////////////////左轮速度/////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,2300); //输出电机PWM  正转
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM 
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2300); //输出电机PWM  正转
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM 
            ///////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
           //   flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
}

/*******************************************************************************
 *  @brief      right_car(void) 
 *  @note       
 *  @warning    
 ******************************************************************************/
void left_car(void)
{           
            speed_now_right = ftm_quad_get(FTM2);  //right轮
            speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left轮
            ftm_quad_clean(FTM2);
            lptmr_pulse_clean();
      
            /*****  Part 1 信息采集 舵机速度PID *****/
            MessageProcessing(); //信息采集
            ADCnormal(); //采集的信息归一化
	    ADCerror_diff(); //偏差法计算 误差 和 误差的变化率
            
 
         // road_check();
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
            speed_fuzzy_mem_cal_forecast(); //对输入的 fe（误差）的绝对值 和 fec（误差变化率）的绝对值 查询隶属度
            speed_fuzzy_query_forecast(); //查询模糊规则表（速度）
            speed_fuzzy_solve_forecast(); //解模糊 输出 speed_forecast 为预期的速度
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get(); 
/**/   //        flag = 0;
/**/    //        dis_right += speed_now_right;
/**/    //        if( dis_right > dis_back ) //倒车后退的距离
/**/    //        {
/**/    //              beep_off();
/**/    //              flag = 1;
/**/    //         }
        //    if(ADC_Normal[0] < 0.1 && ADC_Normal[3] < 0.1 && left_flag == 1)
        //    {
        //        left_flag = 0;
        //    }
  /**/  //    if((ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.6) || left_flag == 1)
  /**/   //  {
  /**/   //       steerctrl = Minsteering;
  /**/   //       left_flag = 1;
  /**/   //   }
           
  /**/      if( ( dis_right > 1500 ) && left_flag == 1 )
  /**/      {
  /**/          flag = 1;
  /**/      }
  /**/      if((ADC_Normal[0] > max_shizi && ADC_Normal[3] > max_shizi) || left_flag == 1)
  /**/      {
  /**/          steerctrl = Maxsteering - 20;
  /**/          left_flag = 1;
                dis_right += speed_now_right;
  /**/      }
            /////////////////////////////舵机///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
            ////////////////////////////////////////////////////////////////////////
            
            ///////////////////////////左轮速度/////////////////////////////////////
            if(0)  //左轮速度溢出  
            {
                if(speedctrl_left < -200) speedctrl_left_opp = 200;
                else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,speedctrl_left_opp); //输出电机PWM  
            }
            else  //左轮速度没溢出
            {   
                if(speedctrl_left < 1500) speedctrl_left = 1500;
                if(speedctrl_left > 1500) speedctrl_left = 1500;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,2300); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////右轮速度////////////////////////////////////
            if(0)  //右轮速度溢出
            {
                if(speedctrl_right < -200) speedctrl_right_opp = 200;
                else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,speedctrl_right_opp); //输出电机PWM  
            }
            else  //右轮速度没溢出
            {
                if(speedctrl_right < 1500) speedctrl_right = 1500;
                if(speedctrl_right > 1500) speedctrl_right = 1500;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2300); //输出电机PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM 
            }
            ///////////////////////////////////////////////////////////////////////
            ////////////////////////////停车///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
            {                                                                                                 
           //   flag = 1;                                                                                       
            }
            if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
            } 
}




