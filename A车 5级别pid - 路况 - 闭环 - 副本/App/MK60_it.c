/*!
 * @file       MK60_it.c
 * @brief      中断服务函数
 * @author     
 * @version    A车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
uint16 times = 0; //定时停车标志位 PIT0定时器
uint8 car_dis_flag = 0; //高电平开始标记位
uint16 car_dis = 0;  //超声波测距距离 单位cm
uint8 car_dis_ms = 0; //超声波测高电平的时间 单位ms
extern uint16 start_flag;
char bluetooth_data=0;//接收到的数据存在这个变量
uint8 car_dis_times = 0;
uint8 now_vol = 0;//现在的电平
uint8 last_vol = 0;//上次的电平
extern uint8 level;
extern uint16 last_stop; 
uint16 delay_flag = 0; //用于周期延时使用
extern uint16 flag;
extern uint16 dis_right;
uint8 wait_flag = 0;
extern float ADC_Normal[5];
uint8 shizi = 0;
extern uint16 dis_back;
extern float speed_power;
uint8 wait_flag_shizi = 2;
uint8 last_flag_shizi = 6;
extern uint8 left_flag;
extern uint8 right_flag;
uint16 gameover = 0;
extern uint8 turn_left_flag;
extern uint8 turn_right_flag;
float last_speed_power = 1;
uint8 is_shizi = 0;
extern uint16 clj;
extern struct _MAG mag_read;
extern uint16 turn_car_dis;
extern uint16 last_start_flag;
uint8 gogogo = 1;
extern int8 ones;
extern uint16 round_is,round_in,round_over,round_num,round_stop_flag,max_PWM,max_PWM_new,round_lr,round_vaule;
uint8 round_times = 0;
/******************************************************************************* 
 *  @brief      PIT0中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   /******  10s 停车  *******/ 
    if(times > 0)  
    {
        times--;
        if(times == 25 && is_shizi == 1)
        {
            shizi++;
            beep_on();
            if( shizi == wait_flag_shizi )  
            {    
                level = 50;
                dis_back = 2000; // 用于十字倒车
                last_stop = 0; //用于停车
                dis_right = 0; //用于倒车
                left_flag = 0;
                right_flag = 0;
            }
            if( shizi == last_flag_shizi )  
            {
                speed_power = last_speed_power; //最后一个十字减速
            }   
        }      
        if( ADC_Normal[4] > 2)
        {
            is_shizi = 0;
        }
        if(times < 1)
        {
            beep_off(); 
        }
    }
    else
    {
        if(level == 1) // 正常模式
        {
            if(ADC_Normal[0] > 0.6 && ADC_Normal[3] > 0.6)
            {
                times = 40;
                is_shizi = 1;
            }
        }
    }  
     
    PIT_Flag_Clear(PIT0);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT1中断服务函数
 *  @note       
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
   //gpio_turn(PTD15); 
    if( start_flag > 0 ) //发车程序，给start_flag赋值就可以进入发车程序
    {
        start_car();
        delay_flag = 100;
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 40 )//终点停车程序
    {
      /*if(last_stop <= 100)  {stop_car();uart_putchar (UART4,'1');}//给last_stop赋值就可以停车
        else 
        {
            if(wait_flag == 0)
            {
                turn_car();
                if( flag == 1 )
                {
                    uart_putchar (UART4,'1');
                    wait_flag = 1;
                }
            }
            else
            {
                //start_flag = 800;
                last_stop = 0; 
                dis_right = 0;
            }
            if(wait_flag == 0)
            {
               // uart_putchar (UART4,'1');
                wait_flag = 1;
                flag = 1;
            }
             
        }*/
        if(last_stop <= 130)  stop_car();//给last_stop赋值就可以停车
        else 
        {
            if(wait_flag == 0)
            {
                turn_car();
                if( speed_power > 0.5 && wait_flag == 0 ) // 高速情况直接发信号 确认是否可以会车
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    speed_power = 0.1;
                }
                if( flag == 1 )  // 倒车完毕 发信号
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    wait_flag = 1;
                }
            }
            else
            {
                //start_flag = 800;
                //last_stop = 0; 
                test_motor();
                flag = 1;
            }
        }    
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 50 )  //十字倒车等待
    {
        if(last_stop <= 50)  stop_car();//给last_stop赋值就可以停车
        else 
        {
            turn_car();
            if(gogogo == 1 && flag == 1)//直接拐
            {
                flag = 0;
                level = 51;
                dis_right = 0;
            }
        }

    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 51 ) //躲避
    {
        if( turn_right_flag == 0)
        {
            right_car();
        }
        if( turn_left_flag == 0)
        {
            left_car();
        }
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 52 ) //重新启动
    {
        turn_car_shizi();
        times = 400; //不计算当前的十字
        is_shizi = 0; //非十字标志
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 100 ) //回赛道停车
    {
        gameover++;
        speed_power = 0.1;
        test_motor();
        if(gameover > 200)  flag = 1;
    }
    ///////////////////////////////////////////////////////////////////////////
    else
    {
        test_motor();
    }
    //test_motor();
   //gpio_turn(PTD15); 
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT2中断服务函数
 *  @note
 *  @warning    超声波
 ******************************************************************************/
void PIT2_IRQHandler(void)
{
  /*
    last_vol = now_vol;
    if( gpio_get(PTC4) == 1 )
        now_vol = 1;
    else 
        now_vol = 0;
    if(last_vol == 0 && now_vol == 1)  //有上升沿
    {
        car_dis_flag = 1;
        car_dis_ms = 0;
    }
      
    if(car_dis_flag == 1)  //超声波测试标记位
    {
        if( car_dis_ms > 150)  //超过 15ms 即为无效数据
            car_dis_flag = 0;
        else
        {
            if( gpio_get(PTC4) == 1 ) //高电平使计数增加5 即高电平时间增加0.5ms
                car_dis_ms += 10;
            else
            { */  
              /*  car_dis_value[car_dis_times] = 34 * car_dis_ms;
                if(car_dis_times == 4) car_dis_times = 0;
                car_dis_times++;
              //  car_dis = 34 * car_dis_ms; //算出超声波距离  mm
                car_dis = car_dis_value[0] + car_dis_value[1] + car_dis_value[2] + car_dis_value[3] + car_dis_value[4];
                car_dis_flag = 0; //重置标记位  */
              /*  if( car_dis < 1300 ) 
		{
                   // if( start_flag < 5) start_flag = 10;
                    //else if(start_flag < 600) start_flag = start_flag * 3;
                    //start_flag = 600;
                    beep_on();
                }
                car_dis_flag = 0;
                car_dis = 34 * car_dis_ms; 
            }
        }      */
    if( round_times != 0)
    {
        round_times--;
        if( round_times == 0)
        {
            round_in=0;
            round_is=0;
            round_over=0;
           // round_num+=1;
            if(level == 4) //误判十字
                level = 88;
            else if(level == 5) //误判十字
                level = 1;
            times = 200; //误判十字
            is_shizi = 0; //误判十字
            round_stop_flag=1;
            max_PWM=max_PWM_new;//恢复pwm限制
            if((round_lr==1)||(round_lr==0))
              round_lr=!round_lr;
        }
    }  
    else if(round_is != 0 && round_vaule != 0)
    {
        round_times = 8;
    }
    //gpio_turn(PTD15);
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


/*!
 *  @brief      UART3测试中断服务函数
 *  @since      v5.0
 *  @warning    蓝牙通讯函数，使用必须开中断才能使用 ,在你需要发送的地方用uart_putchar函数，如果uart_putchar(UART4,'1');
 *  Sample usage:  set_vector_handler(UART3_RX_TX_VECTORn , uart3_test_handler);    //把 uart3_handler 函数添加到中断向量表，不需要我们手动调用
 */
void uart4_test_handler(void)
{

    if(uart_query(UART4) == 1)   //接收数据寄存器满
    {
      uart_getchar(UART4, &bluetooth_data);//等待接收完//*bluetooth_data = UART_D_REG(UARTN[uratn]);///////////bluetooth_data是一个char型变量，你喜欢干啥就干啥
      if(bluetooth_data ==  '1') 
      {
         if( wait_flag == 1 )
         {
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              flag = 0;
              wait_flag = 0;
              start_flag = last_start_flag;
              level = 100;
         }
        // bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '2') 
      {
         flag = 0;
         wait_flag = 0;
         start_flag = last_start_flag;
         level = 100;
        // bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '3')  //躲避
      {
          level = 51;
          flag = 0;
          dis_right = 0;
        // bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '4')  //重新启动
      {
          level = 52;
          flag = 0;
          dis_right = 0;
        // bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '8') 
      {
        // wait_flag_shizi = 0;
        // last_start_flag = 0;
         level = 88;
         flag = 0;
         speed_power = 1;
         //bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
    }


}
