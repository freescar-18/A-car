/*!
 * @file       main.c
 * @brief      main 
 * @author     
 * @version    A车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
extern void  OutPut_Data_test_sscom(void);
uint8 adc_test = 0;
extern uint16 flag ;
extern int length;
extern uint8 chaoshengbotime;
extern uint8 flag_csb;
extern uint32 timevar;
extern byte bmp[];
extern struct _MAG mag_read;
uint16 clj = 0;
extern uint16 last_stop;//终点停车标记 大于1为停车
extern uint8 level;
extern uint16 dis_right,dis_left;
extern uint16 dis_back;
extern uint8 wait_flag;
extern uint16 start_flag;
extern uint16 turn_car_dis;
extern void OutPut_Data_test(void);
/*!
 *  @brief      main函数
 *  @since      
 *  @note       
 */
  void main()
{   
    gpio_init (PTD15, GPO,0); 
    System_Initialization(); //初始化
    ISR_Initialization();  //中断初始化
    
    adc_test = 0; 
    while(adc_test == 0)
    {
        test_max_ADC();
        switch_read();
    }   
    
    
    //初始化PIT1    
    pit_init_ms(PIT1, PIT1_TIMER);                                
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler); //设置PIT1的中断服务函数为 PIT1_IRQHandler
    enable_irq(PIT1_IRQn); // 使能PIT1中断,车子开始动
    set_vector_handler(UART4_RX_TX_VECTORn , uart4_test_handler);//设置中断级别
    uart_rx_irq_en(UART4);//蓝牙窗口中断使能
    

    //enable_irq(PIT0_IRQn); 
    
  /*  gpio_init (PTB18, GPI,0);
    port_init(PTB18, ALT1 | IRQ_RISING | PULLUP );
    set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);
    enable_irq (PORTB_IRQn);*/
  
    
   // 设置中断优先级  越小越优先 15个级别UART4_RX_TX_IRQn
    set_irq_priority(UART4_RX_TX_IRQn,0);
   // set_irq_priority(UART4_ERR_IRQn,1);
    set_irq_priority(PIT2_IRQn,2); 
    set_irq_priority(PORTC_IRQn,3);
    set_irq_priority(PORTB_IRQn,4);
    set_irq_priority(PORTA_IRQn,5);
    set_irq_priority(PORTE_IRQn,6);
    set_irq_priority(PIT1_IRQn,7);
    set_irq_priority(PIT0_IRQn,8);
      
    DisableInterrupts;
    DELAY_MS(10);
    EnableInterrupts; //同时启动中断   
    
    
    while(1)
    {
      //  LED_P6x8Str(0,4,"ojbk");
        oled_view();
     //   test_motor(); 
      //  test_steering();
    //    test_ADC();
     //   test_max_ADC();
     
      //  OutPut_Data_test();//示波器调试 
        /*
        LED_PrintShort(45,2,mag_read.mag_x);
        LED_PrintShort(45,4,mag_read.mag_y); 
       if( mag_read.mag_y < -3000 || (mag_read.mag_y > 500) )
       {
          clj = 1000;
          level = 40;
          dis_back = 1000;
          dis_right = 0;
          last_stop = 0;
       }
       else
       {
          clj = 0;
       }*/
       //  LED_PrintShort(45,5,mag_read.mag_x_offset); 
       //  LED_PrintShort(45,6,mag_read.mag_y_offset);     
     //    Freecars_scope();//多通道示波器调试      
     //    gpio_turn(PTD15);   
   //  OutPut_Data_test_sscom();//串口助手调试  
      //   MessageProcessing(); 
      //   Road_Id_Get();
       //  test_nrf_re();
      //   LED_PrintBMP(0,0,112,1,bmp);
   /*      if((flag_csb == 1)&&(gpio_get(PTB18) == 0))  
          {
            beep_off();
            timevar = pit_time_get(PIT2);
            chaoshengbotime = (0xffffffff-timevar)/50; //ms
            length = chaoshengbotime*340/1000; //mm
            pit_close(PIT2);

           }
            LED_P6x8Str(0,4,"ojbk");
            LED_PrintShort(0,5,length);
            LED_PrintShort(0,6,chaoshengbotime);
           */
      
    }
}