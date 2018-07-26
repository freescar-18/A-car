/*!
 * @file       button.c
 * @brief      按键函数
 * @author     
 * @version    A车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
extern uint16 steering_test,motorctrl_test; //test文件
extern uint16 flag;  //test文件
extern uint16 jishu;  //GetMessage文件
extern uint16 ADC_Maxing[5]; //用于读取flash中存储的最大电感值
extern uint8 adc_test; //跳出最大电感值采集的标志位
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_Rule[5];
extern float speed_error_Rule[5];
extern uint8 huandao_flag_a,huandao_flag_b,huandao_flag_c,huandao_flag_d,huandao_flag_e,huandao_flag_f;
extern uint32 timevar = 0;
extern uint8 chaoshengbotime = 0;
extern int  length = 0;
extern uint8 flag_csb = 0;
int8 ones = 0;//只能写一次数据！！
int8 tab = 0;
extern int8 times; //定时停车标志位 PIT0定时器
extern uint16 last_stop;//终点停车标记 大于1为停车
extern uint8 car_dis_flag; //高电平开始标记位
extern uint16 car_dis;  //超声波测距距离 单位cm
extern uint8 car_dis_ms; //超声波测高电平的时间 单位ms
extern uint16 speed;
extern uint16 delay_flag;
extern uint16 dis_back;
extern uint8 wait_flag;
extern uint8 shizi;
extern uint8 left_flag;
extern uint8 right_flag;
uint8 turn_left_flag = 1;
uint8 turn_right_flag = 1;
extern uint8 switch_mode;
extern uint8 wait_flag_shizi;
extern uint8 last_flag_shizi;
extern float steer_D;
extern float last_speed_power;
extern uint16 max_PWM;
extern uint16 turn_car_dis;
extern uint16 last_start_flag;
extern float max_shizi;
extern uint8 gogogo;
extern float speed_power;
extern float eRule[5];
/////////////////////////////////////////////////////////////////////////////// 
extern uint16 round_vaule;// round_vaule=0       不入环
                       // round_vaule=1       环在左边
                       // round_vaule=2       环在右边
//识别阈值
extern float  round_up_vaule;
extern float round_down_vaule;
//刹车强度
extern uint8 round_stop_vaule;
extern uint8 page_line;
extern uint8 write_flash_flag;
extern uint8 read_flash_flag;
unsigned char switch_data;


/*
extern uint8 n_flag,rhd_n_flag,rhd_n_flag_a,rhd_n_flag_b,rhd_n_flag_c,rhd_n_flag_d,rhd_n_flag_e,rhd_n_flag_f,rhd_n_flag_g;
extern uint8 s_flag,rhd_s_flag,rhd_s_flag_a,rhd_s_flag_b,rhd_s_flag_c,rhd_s_flag_d,rhd_s_flag_e,rhd_s_flag_f,rhd_s_flag_g;
extern uint8 chd_n_flag,chd_n_flag_a,chd_n_flag_b;                              
extern uint8 chd_s_flag,chd_s_flag_a,chd_s_flag_b;  
*/

/*******************************************************************************
 *  @brief      PORT的参考中断服务函数
 *  @since      v5.0
 *  @warning    
 ******************************************************************************/
void PORTA_IRQHandler(void)
{
    uint8  n = 0;    //引脚号
    
    ////////////////////PTA24  DOWN 按键 ///////////////////////////////////////
    n = 24;
    if(PORTA_ISFR & (1 << n))           //PTE0触发中断
    {
        PORTA_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
        if(switch_mode == 100)//初始状态
        {
            if(adc_test  == 0)
            {
                ADC_Maxing[0] = flash_read(SECTOR_NUM, 0, uint16);  //读取16位
                ADC_Maxing[1] = flash_read(SECTOR_NUM, 4, uint16);  //读取16位
                ADC_Maxing[2] = flash_read(SECTOR_NUM, 8, uint16);  //读取16位
                ADC_Maxing[3] = flash_read(SECTOR_NUM, 12, uint16);  //读取16位 2字节
                ADC_Maxing[4] = flash_read(SECTOR_NUM, 16, uint16);  //读取16位 2字节
            }
            adc_test = 1;
            ones = 1;
/**/        start_flag = 0;
            DELAY_MS(300);
        }
        else if(switch_mode == 0)//显示屏0
        {
            if(ones <= 1)
            {
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //输出电机PWM  right-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //输出电机PWM  left-正
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //输出电机PWM  left-反
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //输出电机PWM  right-反
                DELAY_MS(3000);
                start_flag = 150;
                flag = 0;
                level = 1;
            }
            ones = 2;
        }
        else if(switch_mode == 1)//显示屏1
        {
            turn_car_dis -= 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//显示屏2
        {
            wait_flag_shizi--;
            DELAY_MS(300);
        }
        else if(switch_mode == 3)//显示屏3
        { 
            last_flag_shizi--;
            DELAY_MS(300);
        }
        else if(switch_mode == 4)//显示屏4
        {
            if( page_line == 1)
              round_vaule--;
           else if( page_line == 2)
             round_up_vaule -= 0.03;
           else if( page_line == 3)
             round_down_vaule -= 0.03;
           else if( page_line == 4)
             round_stop_vaule -= 3;
           DELAY_MS(300);
        }
        else if(switch_mode == 5)//显示屏5
        {
            max_PWM -= 50;
            DELAY_MS(300);
        }
        else if(switch_mode == 6)//显示屏6
        {
            Rule_kp[0] = Rule_kp[0] + 0.05;
            Rule_kp[1] = Rule_kp[1] + 0.01;
            Rule_kp[3] = Rule_kp[3] - 0.01;
            Rule_kp[4] = Rule_kp[4] - 0.05;
            DELAY_MS(300);
        } 
        else if(switch_mode == 7)//显示屏7
        {
            read_flash();
            DELAY_MS(300);
        }
        else if(switch_mode == 8)//显示屏8
        {
            max_shizi -= 0.1;
            DELAY_MS(300);
        }
        else if(switch_mode == 9)//显示屏9
        {
            eRule[0] = eRule[0] + 1;
            eRule[1] = eRule[1] + 1;
            eRule[3] = eRule[3] - 1;
            eRule[4] = eRule[4] - 1;
            DELAY_MS(300);
        }
        else if(switch_mode == 10)//显示屏10
        {
            speed_Rule[0]--;
            speed_Rule[1]--;
            speed_Rule[2]--;
            speed_Rule[3]--;
            speed_Rule[4]--;
            DELAY_MS(300);
        }
         
        /*  以上为用户任务  */
    }
    
     ///////////////// PTA25 LEFT 按键  //////////////////////////////////////// 
    n = 25;
    if(PORTA_ISFR & (1 << n))           //PTE3触发中断
    {
        PORTA_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
        if(switch_mode == 100)//初始状态
        {
            if(ones == 0)//写入数据
            {
                flash_init();  //初始化flash
                test_max_ADC_flash_write();
            }
            ones = 1;  // 只会写一次！！
            DELAY_MS(300);
        }
        else if(switch_mode == 0)//显示屏0
        {
            
        }
        else if(switch_mode == 1)//显示屏1
        {
            last_start_flag += 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//显示屏2
        {
           turn_left_flag = 0; //左转
           turn_right_flag = 1; //关闭右转
           DELAY_MS(300);
        }
        else if(switch_mode == 3)//显示屏3
        {
           last_speed_power += 0.1;
           DELAY_MS(300);
        }
        else if(switch_mode == 4)//显示屏4
        {
           if( page_line != 1)
              page_line--;
            DELAY_MS(300);
        }
        else if(switch_mode == 5)//显示屏5
        {
           
        }
        else if(switch_mode == 6)//显示屏6
        {
            steer_D += 0.5;
            DELAY_MS(300);
        } 
        else if(switch_mode == 7)//显示屏7
        {
            if( write_flash_flag == 0 )
              write_flash();
            DELAY_MS(300);
        }
        else if(switch_mode == 8)//显示屏8
        {
            gogogo = 1;
            DELAY_MS(300);
        }
        else if(switch_mode == 9)//显示屏9
        {
            
        }
        else if(switch_mode == 10)//显示屏10
        {
           speed_error_Rule[0] += 2;
           speed_error_Rule[1]++;
           speed_error_Rule[2]++;
           DELAY_MS(300);
        }
     }
        
        /*  以上为用户任务  */
    
}
void PORTB_IRQHandler(void)
{
    uint8 n=0; 
    ////////////////////  PTB2 UP 按键  ////////////////////////////////////////
    n = 2;
    if(PORTB_ISFR & (1 << n))           //PTE1触发中断
    {
        PORTB_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
        if(switch_mode == 100)//初始状态
        {
          
        }
        else if(switch_mode == 0)//显示屏0
        {
            //level = 88;
            //flag = 0;
        }
        else if(switch_mode == 1)//显示屏1
        {
            turn_car_dis += 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//显示屏2
        {
            wait_flag_shizi++;
            DELAY_MS(300);
        }
        else if(switch_mode == 3)//显示屏3
        {
            last_flag_shizi++;
            DELAY_MS(300);
        }
        else if(switch_mode == 4)//显示屏4
        {
            if( page_line == 1)
              round_vaule++;
           else if( page_line == 2)
             round_up_vaule += 0.03;
           else if( page_line == 3)
             round_down_vaule += 0.03;
           else if( page_line == 4)
             round_stop_vaule += 3;
           DELAY_MS(300);
        }
        else if(switch_mode == 5)//显示屏5
        {
            max_PWM += 50;
            DELAY_MS(300);
        }
        else if(switch_mode == 6)//显示屏6
        {
            Rule_kp[0] = Rule_kp[0] - 0.05;
            Rule_kp[1] = Rule_kp[1] - 0.01;
            Rule_kp[3] = Rule_kp[3] + 0.01;
            Rule_kp[4] = Rule_kp[4] + 0.05;
            DELAY_MS(300);
        } 
        else if(switch_mode == 7)//显示屏7
        {
            
        }
        else if(switch_mode == 8)//显示屏8
        {
            max_shizi += 0.1;
            DELAY_MS(300);
        }
        else if(switch_mode == 9)//显示屏9
        {
            eRule[0] = eRule[0] - 1;
            eRule[1] = eRule[1] - 1;
            eRule[3] = eRule[3] + 1;
            eRule[4] = eRule[4] + 1;
            DELAY_MS(300);
        }
        else if(switch_mode == 10)//显示屏10
        {
            speed_Rule[0]++;
            speed_Rule[1]++;
            speed_Rule[2]++;
            speed_Rule[3]++;
            speed_Rule[4]++;
            DELAY_MS(300);
        }
        /*  以上为用户任务  */
    }
    /////////////  PTB3 RIGHT 按键   ///////////////////////////////////////////
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTE2触发中断
    {
        PORTB_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
        if(switch_mode == 100)//初始状态
        {
          
        }
        else if(switch_mode == 0)//显示屏0
        {
            
        }
        else if(switch_mode == 1)//显示屏1
        {
            last_start_flag -= 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//显示屏2
        {
           turn_right_flag = 0; //右转
           turn_left_flag = 1; //关闭左转
           DELAY_MS(300);
        }
        else if(switch_mode == 3)//显示屏3
        {
           last_speed_power -= 0.1;
           DELAY_MS(300);
        }
        else if(switch_mode == 4)//显示屏4
        {
           if( page_line != 4)
              page_line++;
            DELAY_MS(300);
        }
        else if(switch_mode == 5)//显示屏4
        {
           
        }
        else if(switch_mode == 6)//显示屏6
        {
            steer_D -= 0.5;
            DELAY_MS(300);
        } 
        else if(switch_mode == 7)//显示屏7
        {
            
        }
        else if(switch_mode == 8)//显示屏8
        {
            gogogo = 0;
            DELAY_MS(300);
        }
        else if(switch_mode == 9)//显示屏9
        {
            
        }
        else if(switch_mode == 10)//显示屏10
        {
           speed_error_Rule[0] -= 2;
           speed_error_Rule[1]--;
           speed_error_Rule[2]--;
           DELAY_MS(300);
        }
        /*  以上为用户任务  */
    }
              
}

void PORTE_IRQHandler(void)
{
    uint8  n = 0;    //引脚号     
     //PTE10 干簧管
    n = 10;
    if(PORTE_ISFR & (1 << n))           //PTE10触发中断
    {
        PORTE_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
        //beep_on();
          if(start_flag == 0 && level != 40 && level!= 100 && level != 86)
          {
              if (level == 88) //自己冲
              {
                  level = 40;
                  dis_back = 0;
                  dis_right = 0;
                  last_stop = 70;
                  wait_flag = 1;
                  round_vaule = 0;
              }
              else
              {
                  level = 40;
                  dis_back = turn_car_dis;
                  dis_right = 0;
                  if( speed_power < 0.5)
                  {
                      last_stop = 90;
                  }
                  else
                  {
                      last_stop = 0;
                  }
                  wait_flag = 0;
                  round_vaule = 0;
              }
             // beep_on();
            
          }
        /*  以上为用户任务  */
    }

}

void PORTC_IRQHandler(void)
{
    uint8  m = 0;    //引脚号
    m = 10;
    if(PORTC_ISFR & (1 << m))           //PTC10触发中断
      {
          PORTC_ISFR  = (1 << m);        //写1清中断标志位   
          nrf_handler();
      } 
    
    ////////////////////////////超声波测距所用参数//////////////////////////////
    m = 4;
    if(PORTC_ISFR & (1 << m))          
      {
          PORTC_ISFR  = (1 << m);        //写1清中断标志位
           /*  以下为用户任务  */
         // car_dis_flag = 1;  //检测到高电平，开启超声波识别位
         // car_dis_ms = 0;
          /*  以上为用户任务  */
      }
    m = 2;
    if(PORTC_ISFR & (1 << m))          
      {
          PORTC_ISFR  = (1 << m);        //写1清中断标志位
           /*  以下为用户任务  */
         /* if(start_flag == 0 && level != 40 && level!= 100 && level != 86)
          {
              if (level == 88) //自己冲
              {
                  level = 40;
                  dis_back = 0;
                  dis_right = 0;
                  last_stop = 90;
                  wait_flag = 1;
              }
              else
              {
                  level = 40;
                  dis_back = turn_car_dis;
                  dis_right = 0;
                  if( speed_power < 0.5)
                  {
                      last_stop = 80;
                  }
                  else
                  {
                      last_stop = 0;
                  }
                  wait_flag = 0;
              }
             // beep_on();
            
          }*/
          /*  以上为用户任务  */
      }
}

