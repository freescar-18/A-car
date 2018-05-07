/*!
 * @file       MK60_it.c
 * @brief      中断服务函数
 * @author     
 * @version    B车
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
extern uint16 ADC_Maxing[4]; //用于读取flash中存储的最大电感值
extern uint8 adc_test; //跳出最大电感值采集的标志位
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_Rule[5];
extern uint8 huandao_flag_a,huandao_flag_b,huandao_flag_c,huandao_flag_d,huandao_flag_e,huandao_flag_f;
extern uint32 timevar = 0;
extern uint8 chaoshengbotime = 0;
extern int  length = 0;
extern uint8 flag_csb = 0;
int8 times = 0; //定时停车标志位 PIT0定时器
int8 ones = 0;//只能写一次数据！！
int8 tab = 0;


/******************************************************************************* 
 *  @brief      PIT0中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   /******  10s 停车  *******/
  /* if(times < 5)  //参数即为时间  如 tiems < 10 ，为 10s 自动停车
    {
      times++;
    }
    else
    {
        flag = 1;
    }
  */
    
    PIT_Flag_Clear(PIT0);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT1中断服务函数
 *  @note       
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
 //  gpio_turn(PTD15); 
    test_motor(); 
 //  gpio_turn(PTD15); 
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT2中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT2_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT2);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT3中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT3);       //清中断标志位
}

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
        if(adc_test  == 0)
        {
            ADC_Maxing[0] = flash_read(SECTOR_NUM, 0, uint16);  //读取16位
            ADC_Maxing[1] = flash_read(SECTOR_NUM, 4, uint16);  //读取16位
            ADC_Maxing[2] = flash_read(SECTOR_NUM, 8, uint16);  //读取16位
            ADC_Maxing[3] = flash_read(SECTOR_NUM, 12, uint16);  //读取16位 2字节
        }
        adc_test = 1;
        ones = 1;
        if(tab == 0) tab = 1;//切换键
        else tab = 0;
        DELAY_MS(300);
         
        /*  以上为用户任务  */
    }
    
     ///////////////// PTA25 LEFT 按键  //////////////////////////////////////// 
    n = 25;
    if(PORTA_ISFR & (1 << n))           //PTE3触发中断
    {
        PORTA_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
       //motorctrl_test = motorctrl_test + 50;
       // steering_test = steering_test + 2;
        if(ones == 0)
        {
            flash_init();  //初始化flash
            test_max_ADC_flash_write();
        }
        ones = 1;  // 只会写一次！！
        
        if(tab == 0)
        {
        Rule_kp[0] = Rule_kp[0] - 0.1;
        Rule_kp[1] = Rule_kp[1] - 0.1;
        }
        if(tab == 1)
        {
        Rule_kd[0] = Rule_kd[0] - 0.01 *10;
        Rule_kd[1] = Rule_kd[1] - 0.01 *10;
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
         flag = 0;
          ones = 1;
         jishu = 0;
         times = 0;//清空停车时间计时
         huandao_flag_a = 0; huandao_flag_b = 0;//huandao_flag_c = 0; 
         huandao_flag_d = 0; huandao_flag_e = 0; //huandao_flag_f = 0;
         DELAY_MS(300);
        /*  以上为用户任务  */
    }
    /////////////  PTB3 RIGHT 按键   ///////////////////////////////////////////
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTE2触发中断
    {
        PORTB_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
      //  motorctrl_test = motorctrl_test - 50;
      //  steering_test = steering_test - 2;
        
        if(tab == 0)
        {
          Rule_kp[4] = Rule_kp[4] + 0.1;
          Rule_kp[3] = Rule_kp[3] + 0.1; 
        }
        if(tab == 1)
        {
          Rule_kd[4] = Rule_kd[4] + 0.01 * 10;
          Rule_kd[3] = Rule_kd[3] + 0.01 * 10; 
        } 
         DELAY_MS(300);
        /*  以上为用户任务  */
    }
    
      
    n=18;
    if(PORTB_ISFR & (1 << n))          
      {
          PORTB_ISFR  = (1 << n);        //写1清中断标志位
           // beep_on();
            
          if(gpio_get(PTB18))
          {
            beep_on();
            pit_time_start(PIT2);
            flag_csb = 1;
          }
          else
          {
            
          }          
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
}



