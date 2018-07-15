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
extern uint16 start_flag;
extern uint8 level;
extern uint16 dis_right,dis_left;
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
        }
        else if(switch_mode == 0)//显示屏0
        {
        
        }
        else if(switch_mode == 1)//显示屏1
        {
            
        }
        else if(switch_mode == 2)//显示屏2
        {
            wait_flag_shizi--;
        }
        else if(switch_mode == 3)//显示屏3
        {
            last_flag_shizi--;
        }
        else if(switch_mode == 4)//显示屏4
        {
            speed_Rule[0]--;
            speed_Rule[1]--;
            speed_Rule[2]--;
            speed_Rule[3]--;
            speed_Rule[4]--;
        }
        else if(switch_mode == 5)//显示屏5
        {
            max_PWM -= 50;
        }
        else if(switch_mode == 6)//显示屏6
        {
            Rule_kp[0] = Rule_kp[0] + 0.05;
            Rule_kp[1] = Rule_kp[1] + 0.05;
            Rule_kp[3] = Rule_kp[3] - 0.05;
            Rule_kp[4] = Rule_kp[4] - 0.05;
        } 
        else if(switch_mode == 7)//显示屏7
        {
            read_flash();
        }
        DELAY_MS(300);
         
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
        }
        else if(switch_mode == 0)//显示屏0
        {
            turn_left_flag = 0;
            //left_flag = 0;
            level = 51;
            flag = 0;
        }
        else if(switch_mode == 1)//显示屏1
        {
            
        }
        else if(switch_mode == 2)//显示屏2
        {
           turn_left_flag = 0; //左转
           turn_right_flag = 1; //关闭右转
        }
        else if(switch_mode == 3)//显示屏3
        {
           last_speed_power += 0.1;
        }
        else if(switch_mode == 4)//显示屏4
        {
           speed_error_Rule[0] += 2;
           speed_error_Rule[1]++;
           speed_error_Rule[2]++;
        }
        else if(switch_mode == 5)//显示屏5
        {
           
        }
        else if(switch_mode == 6)//显示屏6
        {
            steer_D += 0.5;
        } 
        else if(switch_mode == 7)//显示屏7
        {
            write_flash();
        }
        DELAY_MS(300); 
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
            level = 52;
            flag = 0;
        }
        else if(switch_mode == 1)//显示屏1
        {
            
        }
        else if(switch_mode == 2)//显示屏2
        {
            wait_flag_shizi++;
        }
        else if(switch_mode == 3)//显示屏3
        {
            last_flag_shizi++;
        }
        else if(switch_mode == 4)//显示屏4
        {
            speed_Rule[0]++;
            speed_Rule[1]++;
            speed_Rule[2]++;
            speed_Rule[3]++;
            speed_Rule[4]++;
        }
        else if(switch_mode == 5)//显示屏4
        {
            max_PWM += 50;
        }
        else if(switch_mode == 6)//显示屏6
        {
            Rule_kp[0] = Rule_kp[0] - 0.05;
            Rule_kp[1] = Rule_kp[1] - 0.05;
            Rule_kp[3] = Rule_kp[3] + 0.05;
            Rule_kp[4] = Rule_kp[4] + 0.05;
        } 
        else if(switch_mode == 7)//显示屏7
        {
            
        }
         DELAY_MS(300);
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
            turn_right_flag = 0;
            level = 51;
            flag = 0;
        }
        else if(switch_mode == 1)//显示屏1
        {
            
        }
        else if(switch_mode == 2)//显示屏2
        {
           turn_right_flag = 0; //右转
           turn_left_flag = 1; //关闭左转
        }
        else if(switch_mode == 3)//显示屏3
        {
           last_speed_power -= 0.1;
        }
        else if(switch_mode == 4)//显示屏4
        {
           speed_error_Rule[0] -= 2;
           speed_error_Rule[1]--;
           speed_error_Rule[2]--;
        }
        else if(switch_mode == 5)//显示屏4
        {
           
        }
        else if(switch_mode == 6)//显示屏6
        {
            steer_D -= 0.5;
        } 
        else if(switch_mode == 7)//显示屏7
        {
            
        }
        DELAY_MS(300);
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
        if(start_flag == 0 && level != 40 && level != 100)
        {
/**/    //    level = 40;
/**/    //    dis_back = 1000;
/**/     //   last_stop = 0;
/**/    //    dis_right = 0;
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
}

