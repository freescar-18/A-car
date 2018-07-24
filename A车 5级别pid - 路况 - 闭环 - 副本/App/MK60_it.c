/*!
 * @file       MK60_it.c
 * @brief      �жϷ�����
 * @author     
 * @version    A��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
uint16 times = 0; //��ʱͣ����־λ PIT0��ʱ��
uint8 car_dis_flag = 0; //�ߵ�ƽ��ʼ���λ
uint16 car_dis = 0;  //������������ ��λcm
uint8 car_dis_ms = 0; //��������ߵ�ƽ��ʱ�� ��λms
extern uint16 start_flag;
char bluetooth_data=0;//���յ������ݴ����������
uint8 car_dis_times = 0;
uint8 now_vol = 0;//���ڵĵ�ƽ
uint8 last_vol = 0;//�ϴεĵ�ƽ
extern uint8 level;
extern uint16 last_stop; 
uint16 delay_flag = 0; //����������ʱʹ��
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
 *  @brief      PIT0�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   /******  10s ͣ��  *******/ 
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
                dis_back = 2000; // ����ʮ�ֵ���
                last_stop = 0; //����ͣ��
                dis_right = 0; //���ڵ���
                left_flag = 0;
                right_flag = 0;
            }
            if( shizi == last_flag_shizi )  
            {
                speed_power = last_speed_power; //���һ��ʮ�ּ���
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
        if(level == 1) // ����ģʽ
        {
            if(ADC_Normal[0] > 0.6 && ADC_Normal[3] > 0.6)
            {
                times = 40;
                is_shizi = 1;
            }
        }
    }  
     
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT1�жϷ�����
 *  @note       
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
   //gpio_turn(PTD15); 
    if( start_flag > 0 ) //�������򣬸�start_flag��ֵ�Ϳ��Խ��뷢������
    {
        start_car();
        delay_flag = 100;
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 40 )//�յ�ͣ������
    {
      /*if(last_stop <= 100)  {stop_car();uart_putchar (UART4,'1');}//��last_stop��ֵ�Ϳ���ͣ��
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
        if(last_stop <= 130)  stop_car();//��last_stop��ֵ�Ϳ���ͣ��
        else 
        {
            if(wait_flag == 0)
            {
                turn_car();
                if( speed_power > 0.5 && wait_flag == 0 ) // �������ֱ�ӷ��ź� ȷ���Ƿ���Իᳵ
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    speed_power = 0.1;
                }
                if( flag == 1 )  // ������� ���ź�
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
    else if( level == 50 )  //ʮ�ֵ����ȴ�
    {
        if(last_stop <= 50)  stop_car();//��last_stop��ֵ�Ϳ���ͣ��
        else 
        {
            turn_car();
            if(gogogo == 1 && flag == 1)//ֱ�ӹ�
            {
                flag = 0;
                level = 51;
                dis_right = 0;
            }
        }

    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 51 ) //���
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
    else if( level == 52 ) //��������
    {
        turn_car_shizi();
        times = 400; //�����㵱ǰ��ʮ��
        is_shizi = 0; //��ʮ�ֱ�־
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 100 ) //������ͣ��
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
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT2�жϷ�����
 *  @note
 *  @warning    ������
 ******************************************************************************/
void PIT2_IRQHandler(void)
{
  /*
    last_vol = now_vol;
    if( gpio_get(PTC4) == 1 )
        now_vol = 1;
    else 
        now_vol = 0;
    if(last_vol == 0 && now_vol == 1)  //��������
    {
        car_dis_flag = 1;
        car_dis_ms = 0;
    }
      
    if(car_dis_flag == 1)  //���������Ա��λ
    {
        if( car_dis_ms > 150)  //���� 15ms ��Ϊ��Ч����
            car_dis_flag = 0;
        else
        {
            if( gpio_get(PTC4) == 1 ) //�ߵ�ƽʹ��������5 ���ߵ�ƽʱ������0.5ms
                car_dis_ms += 10;
            else
            { */  
              /*  car_dis_value[car_dis_times] = 34 * car_dis_ms;
                if(car_dis_times == 4) car_dis_times = 0;
                car_dis_times++;
              //  car_dis = 34 * car_dis_ms; //�������������  mm
                car_dis = car_dis_value[0] + car_dis_value[1] + car_dis_value[2] + car_dis_value[3] + car_dis_value[4];
                car_dis_flag = 0; //���ñ��λ  */
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
            if(level == 4) //����ʮ��
                level = 88;
            else if(level == 5) //����ʮ��
                level = 1;
            times = 200; //����ʮ��
            is_shizi = 0; //����ʮ��
            round_stop_flag=1;
            max_PWM=max_PWM_new;//�ָ�pwm����
            if((round_lr==1)||(round_lr==0))
              round_lr=!round_lr;
        }
    }  
    else if(round_is != 0 && round_vaule != 0)
    {
        round_times = 8;
    }
    //gpio_turn(PTD15);
    PIT_Flag_Clear(PIT2);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT3�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT3);       //���жϱ�־λ
}


/*!
 *  @brief      UART3�����жϷ�����
 *  @since      v5.0
 *  @warning    ����ͨѶ������ʹ�ñ��뿪�жϲ���ʹ�� ,������Ҫ���͵ĵط���uart_putchar���������uart_putchar(UART4,'1');
 *  Sample usage:  set_vector_handler(UART3_RX_TX_VECTORn , uart3_test_handler);    //�� uart3_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */
void uart4_test_handler(void)
{

    if(uart_query(UART4) == 1)   //�������ݼĴ�����
    {
      uart_getchar(UART4, &bluetooth_data);//�ȴ�������//*bluetooth_data = UART_D_REG(UARTN[uratn]);///////////bluetooth_data��һ��char�ͱ�������ϲ����ɶ�͸�ɶ
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
        // bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '2') 
      {
         flag = 0;
         wait_flag = 0;
         start_flag = last_start_flag;
         level = 100;
        // bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '3')  //���
      {
          level = 51;
          flag = 0;
          dis_right = 0;
        // bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '4')  //��������
      {
          level = 52;
          flag = 0;
          dis_right = 0;
        // bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '8') 
      {
        // wait_flag_shizi = 0;
        // last_start_flag = 0;
         level = 88;
         flag = 0;
         speed_power = 1;
         //bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
    }


}
