/*!
 * @file       test.c
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
// test_ADC函数的定义
int16 count_test = 0,flag_test = 0,error_test = 0,error_test2 = 0; 
int16 var_test2 = 0,var_test3 = 0,var_test5 = 0,var_test6 = 0,lptmr_test = 0;
double error_sqrt_test = 0,var_sqrt_test3 = 0,var_sqrt_test5 = 0,error_test3 = 0;
// test_motor函数的定义    
extern uint16 ADC_GetMessage[4][SamplingNum];
extern uint16 ADC_Value[4];
extern uint16 SUM_ADC_GetMessage[4];
extern uint16 ADC_Maxing[4];
extern float ADC_Normal[4];
extern float fe,fec;
extern float speed_fec,speed_fe;
extern float steer_P;
extern int16 steerctrl;
extern float speed_forecast;
extern float speed_forecast_error;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
int16 speed_now_left,speed_now_right;


float speed_min = 2000; //用于记录最小速度
float speed_fec_max = 0; //用于记录最大误差变化率
int8 u = 0;
//uint8 flag = 0;  //用于停车的标志位 
                 //如果电感值都变小 则 flag 变 1，进入while（flag == 1）死循环中 
                 //按下按键即可将 flag 变 0，跳出死循环重新启动
                 //

//test_steering函数的定义
uint16 steering_test = 750;  //测试时所用的舵机PWM
uint32 ADC_max_test[4] = {0,0,0,0};//测试最大电感值所用
uint8 w;

extern float speed_power;
extern uint8 none_steerctrl; 



/*******************************************************************************
 *  @brief      test_ADC函数 
 *  @note       ADC显示测试函数，用于前期测试电感值，编码器，OLED显示  
                直接放入main中while（1)里执行
                下面为oled显示的参数

              0                     竖着的电感差值
              1   绿色电感值
              2   蓝色电感值           蓝色电感值开更
              3  横着两个电感的差值   横着两个电感值开更后的差值
              4   褐色电感值           褐色电感值开更
              5   橙色电感值
              6
              7     左编码器值             右编码器值
        
 *  @warning    18/3/9  v2.0
 ******************************************************************************/
void test_ADC(void)
{       
    //LED_P6x8Str(0,0,"PTB4:"); //1
    LED_P6x8Str(0,1,"GREEN:");
    LED_P6x8Str(0,2,"BLUE:");  //3
    //LED_P6x8Str(0,3,"PTB7:");
    LED_P6x8Str(0,4,"BROWN:");  //5
    LED_P6x8Str(0,5,"ORANGE:");
      
    count_test = ftm_quad_get(FTM2);  //
    lptmr_test = lptmr_pulse_get();
    //var_test1 = adc_once(ADC1_SE10, ADC_12bit);
    var_test2 = adc_once(ADC1_SE10, ADC_12bit);
    var_test3 = adc_once(ADC1_SE13, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
    var_test5 = adc_once(ADC1_SE14, ADC_12bit);
    var_test6 = adc_once(ADC1_SE15, ADC_12bit);
    
    error_test = var_test3 - var_test5;
    error_test2 = var_test2 - var_test6;
    var_sqrt_test3 = sqrt(var_test3); //开更平缓
    var_sqrt_test5 = sqrt(var_test5);
    error_sqrt_test = var_sqrt_test3 - var_sqrt_test5;
    //error_test3 = var_test3 / var_test5;
    
    if(count_test >= 10000)
    {
      flag_test++;
      ftm_quad_clean(FTM2);     
    }
    
    //LED_PrintShort(70,0,var_test1);
    LED_PrintShort(35,1,var_test2);
    LED_PrintShort(30,2,var_test3);
   // LED_PrintShort(33,3,var_test4);
    LED_PrintShort(35,4,var_test5);  
    LED_PrintShort(40,5,var_test6);
    LED_PrintShort(70,7,count_test);
    LED_PrintShort(0,7,lptmr_test);
    LED_PrintShort(15,3,error_test);
    LED_PrintShort(75,0,error_test2);
    //LED_PrintValueF(75,1,error_test3,1);
    LED_PrintValueF(75,3,error_sqrt_test,1);
    LED_PrintValueF(85,2,var_sqrt_test3,1);
    LED_PrintValueF(85,4,var_sqrt_test5,1);
    DELAY_MS(100);
}

/*******************************************************************************
 *  @brief      test_motor函数 
 *  @note       直接放入main中while（1)里执行
                需要在MK60_it里添加按键的中断
                up ：flag = 0；
                
                该函数分为6个 Part 可以执行自动停车、进行简单的电感采集处理和模糊PID处理、无路况识别
                按 up 键跳出停车，车轮继续转
               
 *  @warning    18/3/15 v4.0  v3.1版本为旧版
 ******************************************************************************/
void test_motor(void)
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
        if(none_steerctrl == 0)
        {
            fuzzy_mem_cal(); //对输入的 fe（误差） 和 fec（误差变化率） 查询隶属度
            fuzzy_query(); //查询模糊规则表
            fuzzy_solve(); //解模糊
            steercontrol(); //舵机控制，输出 steerctrl 为舵机转角
        }
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
     
        /***************  没有闭环的补充 **********************/
    /*    if(fe < 0)
        {
            speedctrl_right = (int)( speed_power * (100 * (speed_forecast - speed_forecast_error)) ); //- speed_forecast_error
            speedctrl_left = (int)( speed_power * (100 * speed_forecast) );
        }
        else
        {
            speedctrl_left = (int)( speed_power * (100 * (speed_forecast - speed_forecast_error)) );
            speedctrl_right = (int)( speed_power * (100 * speed_forecast) );
        } */
         /*****************************************************/
       
      //  if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
      //  if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
        
        /****** 记录有用的数据 *****/
        if(u<3) u++; //由于第一次计算没有上一次的值，因此避开前几次的数据记录
        else
        {
        if(speed_fec > speed_fec_max)  speed_fec_max = speed_fec; //记录最大fec
        if(speed_forecast < speed_min)  speed_min = speed_forecast; //记录最小速度
        }
        
        /*****  Part 4 停车判断  *****/
       if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //如果四个电感都偏小，则将flag变成1，然后进入下面的死循环
        {                                                                                                 
          flag = 1;                                                                                       
        }     
     /*   if(flag == 1) // 进入死循环，电机停止转动
        {
         speed_forecast = 0;
          //   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //电机输出 0
         //   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //电机输出 0
       //     LED_PrintShort(50,7,speed_forecast); //显示电机PWM
        //  LED_PrintValueF(50,5,speed_fec_max,2); //显示最大误差变化率的绝对值
        //     LED_PrintValueF(50,3,speed_min,2); //显示最小速度
        //     DELAY_MS(100);
             
        }*/
       
        if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //舵机转角保护
        if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //舵机转角保护
        
        if(speedctrl_right < 200) speedctrl_right = 200;
        if(speedctrl_left < 200) speedctrl_left = 200;
        if(speedctrl_left > 4500) speedctrl_left = 4500;
        if(speedctrl_right > 4500) speedctrl_right = 4500;
        
        if( flag == 1 ) // 进入死循环，电机停止转动，此时因为几十毫秒过去了，还是这么小，所以就停车了
        {          
            speedctrl_left = 0;
            speedctrl_right = 0;   
        }        
        ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //输出舵机PWM
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,speedctrl_right); //输出电机PWM
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,speedctrl_left); //输出电机PWM   左电机       
}

/*******************************************************************************
 *  @brief      test_steering函数
 *  @note       直接放入main中while（1)里执行
                需要在MK60_it里添加按键的中断
                left ：steering_test = steering_test + 5;
                right ：steering_test = steering_test - 5;
      
                测试舵机转角函数，oled显示舵机输出的PWM,按left增加舵机PWM，按right减小舵机PWM

 *  @warning    18/3/11 v4.0
 ******************************************************************************/
void test_steering(void)
{
      ftm_pwm_duty(S3010_FTM, S3010_CH,steering_test);  //舵机转角
      LED_PrintShort(50,7,steering_test);  //显示舵机PWM
}

/*******************************************************************************
 *  @brief      test_max_ADC函数
 *  @note       检测最大电感采集
                oled显示
                

 *  @warning    18/3/19
 ******************************************************************************/
void test_max_ADC(void)
{
    ADC_GetMessage[0][1] = adc_once(ADC1_SE10, ADC_12bit); //Green
    ADC_GetMessage[1][1] = adc_once(ADC1_SE13, ADC_12bit); //blue
    ADC_GetMessage[2][1] = adc_once(ADC1_SE14, ADC_12bit); //brown
    ADC_GetMessage[3][1] = adc_once(ADC1_SE15, ADC_12bit);  //orange
    
    for(w = 0;w < 4; w++)
    {
        if( ADC_GetMessage[w][1] >= ADC_max_test[w])
            ADC_max_test[w] = ADC_GetMessage[w][1];
    }
    
    LED_PrintShort(0,0,ADC_max_test[0]);  //显示绿色电感值
    LED_PrintShort(0,1,ADC_max_test[1]);  //显示蓝色电感值
    LED_PrintShort(0,2,ADC_max_test[2]);  //显示褐色电感值
    LED_PrintShort(0,3,ADC_max_test[3]);  //显示橙色电感值
            
}

/*******************************************************************************
 *  @brief      test_max_ADC_flash_write函数
 *  @note       将
                

 *  @warning    18/3/19
 ******************************************************************************/
void test_max_ADC_flash_write(void)
{
    flash_erase_sector(SECTOR_NUM);                     //擦除扇区
                                                        //写入flash数据前，需要先擦除对应的扇区(不然数据会乱)
    flash_write(SECTOR_NUM, 0, ADC_max_test[0] );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 4, ADC_max_test[1] );   //写入数据到扇区，偏移地址为4，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 8, ADC_max_test[2] ) ;  //写入数据到扇区，偏移地址为8，必须一次写入4字节
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 12, ADC_max_test[3] ) ;  //写入数据到扇区，偏移地址为12，必须一次写入4字节
    DELAY_MS(50);
}


/*******************************************************************************
 *  @brief      test_nrf_tr  函数
 *  @note       
                

 *  @warning    18/4/7
 ******************************************************************************/
void test_nrf_tr(void)
{
  uint32 i=0;
  uint8 buff[DATA_PACKET]={0}; 
  uint8 j = 0;
  
  while(!nrf_init())
  {}
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler); //设置 PORTE 的中断服务函数为 PORTE_VECTORn 21 enable_irq(PORTE_IRQn); 
  enable_irq (PORTC_IRQn);
  beep_on();DELAY_MS(10);beep_off();
  
  while(1) 
  {      
    if(nrf_tx(buff,DATA_PACKET) == 1 ) //发送一个数据包：buff（包为32字节）  
    {//等待发送过程中，此处可以加入处理任务      
        while(nrf_tx_state() == NRF_TXING); //等待发送完成  
        if( NRF_TX_OK == nrf_tx_state () ) 
        { 
             //beep_on();DELAY();beep_off();
            
          buff[0] = j ; j++; 
          if(j>=10) j=0; 
        }
       else 
         { 
           printf("\n发送失败:%d",i); 
          } 
     } 
    else 
    {
      printf("\n发送失败:%d",i);
    }
        //DELAY();
  }
}
/*******************************************************************************
 *  @brief      test_nrf_re 函数
 *  @note       
                

 *  @warning    18/4/7
 ******************************************************************************/
void test_nrf_re(void) 
{ 
   uint8 buff[DATA_PACKET]; //定义接收缓冲区 
   uint32 relen;  

   while(!nrf_init()) //初始化NRF24L01+ ,等待初始化成功为止 
   {  }
   //配置中断服务函数 
   set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler); //设置 PORTE 的中断服务函数为 PORTE_VECTORn
   enable_irq(PORTC_IRQn);
     beep_on();DELAY_MS(10);beep_off();
 
   while(1) 
  { 
       relen = nrf_rx(buff,DATA_PACKET); //等待接收一个数据包，数据存储在buff里 
       if(relen != 0) 
       { 
            LED_PrintShort(0,5,buff[0]);
       }
   } 
 }