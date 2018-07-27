/*!
 * @file       main.c
 * @brief      main 
 * @author     
 * @version    A��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
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
extern uint16 last_stop;//�յ�ͣ����� ����1Ϊͣ��
extern uint8 level;
extern uint16 dis_right,dis_left;
extern uint16 dis_back;
extern uint8 wait_flag;
extern uint16 start_flag;
extern uint16 turn_car_dis;
extern void OutPut_Data_test(void);
/*!
 *  @brief      main����
 *  @since      
 *  @note       
 */
  void main()
{   
    gpio_init (PTD15, GPO,0); 
    System_Initialization(); //��ʼ��
    ISR_Initialization();  //�жϳ�ʼ��
    
    adc_test = 0; 
    while(adc_test == 0)
    {
        test_max_ADC();
        switch_read();
    }   
    
    
    //��ʼ��PIT1    
    pit_init_ms(PIT1, PIT1_TIMER);                                
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler); //����PIT1���жϷ�����Ϊ PIT1_IRQHandler
    enable_irq(PIT1_IRQn); // ʹ��PIT1�ж�,���ӿ�ʼ��
    set_vector_handler(UART4_RX_TX_VECTORn , uart4_test_handler);//�����жϼ���
    uart_rx_irq_en(UART4);//���������ж�ʹ��
    

    //enable_irq(PIT0_IRQn); 
    
  /*  gpio_init (PTB18, GPI,0);
    port_init(PTB18, ALT1 | IRQ_RISING | PULLUP );
    set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);
    enable_irq (PORTB_IRQn);*/
  
    
   // �����ж����ȼ�  ԽСԽ���� 15������UART4_RX_TX_IRQn
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
    EnableInterrupts; //ͬʱ�����ж�   
    
    
    while(1)
    {
      //  LED_P6x8Str(0,4,"ojbk");
        oled_view();
     //   test_motor(); 
      //  test_steering();
    //    test_ADC();
     //   test_max_ADC();
     
      //  OutPut_Data_test();//ʾ�������� 
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
     //    Freecars_scope();//��ͨ��ʾ��������      
     //    gpio_turn(PTD15);   
   //  OutPut_Data_test_sscom();//�������ֵ���  
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