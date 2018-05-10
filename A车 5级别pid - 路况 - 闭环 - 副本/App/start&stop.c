/*!
 * @file       start&stop.c
 * @brief      存放各类之前测试的函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
extern float ADC_Normal[4];
extern uint16 ADC_Value[4];
extern float fe,fec;
extern int16 steerctrl;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
extern int16 speed_now_left,speed_now_right;
extern float speed_power;
extern uint8 none_steerctrl; 
extern int16 speedctrl_left,speedctrl_right; 
extern int16 speedctrl_left_opp,speedctrl_right_opp; 
extern uint8 last_stop;  //终点停车标记位  0为不是停车
extern float speed_forecast; //预测将要达到的速度（PWM）
extern float speed_forecast_error; //预测将要达到的速度的偏差（差速）
extern float fe,fe1,fe2,fe_last;
uint16 start_flag = start_Seconds;

/*******************************************************************************
 *  @brief      start_car(void) 
 *  @note       
 *  @warning    
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
            
  /**/      if(ADC_Normal[2] < 0.3  && ADC_Normal[1] < 0.05)  steerctrl = Maxsteering; //左电感太小，向左打角
  /**/      else if(ADC_Normal[2] > 0.1 && ADC_Normal[1] > 0.6)  steerctrl = Minsteering; //右电感太大，向右打角

            
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
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,speedctrl_left); //输出电机PWM  
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
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,speedctrl_right); //输出电机PWM  
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



