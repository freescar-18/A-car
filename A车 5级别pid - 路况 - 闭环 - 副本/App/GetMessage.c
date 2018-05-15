/*!
 * @file       GetMessage.c
 * @brief      ���ݲɼ�����
 * @author     
 * @version    A��
 * @date       
 */
   
/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_power;
uint16 ADC_GetMessage[4][SamplingNum]; //�ɼ������ĵ��ֵ��һ����й� SamplingNum ��
uint16 ADC_Value[4] = {0,0,0,0}; //�˲�ȡƽ����ĵ��ֵ
uint16 SUM_ADC_GetMessage[4] = {0,0,0,0}; 
uint16 ADC_Maxing[4] = {3300,3300,3300,3300}; //��е����ֵ��������Ҫ�⣩
float ADC_Normal[4] = {0,0,0,0}; //��й�һ�����ֵ����Χ 0~1 ��
float fe = 0; //��������
float fe1,fe2;
float fe_last; //��һ�ε����
float fec; //���ı仯��
float error_diff_c;
float error_diff;
float error_diff_last;
extern float P_power;
extern float D_power;
uint16 i,j,k;
uint16 jishu=0;
uint16 change; //���ڽ�������ʱ���õ��н�
uint8 cross_rank = 0,line_rank = 0;
uint8 flag = 0;    //����ͣ���ı�־λ 
                  //������ֵ����С �� flag �� 1������while��flag == 1����ѭ���� 
                 //���°������ɽ� flag �� 0��������ѭ����������
                 //


float ADC_Yuanhuan_L1=0.000,ADC_Yuanhuan_L2=0.000,ADC_Yuanhuan_L3=0.000,ADC_Yuanhuan_L4=0.000,ADC_Yuanhuan_L5=0.000;

extern int16 steerctrl;
extern int16 last_steerctrl;
extern int16 speed_forecast;

uint8 level;
float dreams = 0.07;
uint16 cross = 0;
uint16 cross_pass = 0;
uint8 none_steerctrl = 0; 
uint16 cross_left = 0;
uint cross_right = 0;

uint8 none_speedctrl = 0; 

uint8 rhd_n_flag=0,rhd_n_flag_a=0,rhd_n_flag_b=0,rhd_n_flag_c=0,rhd_n_flag_d=0,rhd_n_flag_e=0,rhd_n_flag_f=0,rhd_n_flag_g=0;  //�����뻷����־
uint8 rhd_s_flag=0,rhd_s_flag_a=0,rhd_s_flag_b=0,rhd_s_flag_c=0,rhd_s_flag_d=0,rhd_s_flag_e=0,rhd_s_flag_f=0,rhd_s_flag_g=0;  //˳���뻷����־
uint8 chd_n_flag=0,chd_n_flag_a=0,chd_n_flag_b=0;                                //�����������־
uint8 chd_s_flag=0,chd_s_flag_a=0,chd_s_flag_b=0;                                //˳���������־
int Island_length = 0; 


/*******************************************************************************
 *  @brief      MessageProcessing����
 *  @note       ADC��Ϣ�ɼ������޹�һ�� 
                ���Ϊÿ����вɼ�SamplingNum�κ��ƽ��ֵ��ȥ����β��
 *  @warning    18/3/9 v3.1
 ******************************************************************************/
void MessageProcessing(void)
{  
    for(i = 0;i < SamplingNum; i++)//�ɼ����SamplingNum��
    {   
        //var_test1 = adc_once(ADC1_SE10, ADC_12bit);
        ADC_GetMessage[0][i] = adc_once(ADC1_SE11, ADC_12bit); //Green
        ADC_GetMessage[1][i] = adc_once(ADC1_SE12, ADC_12bit); //blue
        //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
        ADC_GetMessage[2][i] = adc_once(ADC1_SE14, ADC_12bit); //brown
        ADC_GetMessage[3][i] = adc_once(ADC1_SE15, ADC_12bit);  //orange
    }
    
    for(i = 0;i < (SamplingNum - 1); i++)  //ð�ݷ����� ��С����
        for(j = i + 1;j < SamplingNum; j++)
            for(k = 0;k < 4; k++)  //ѡ����
            {
                if( ADC_GetMessage[k][i] >= ADC_GetMessage[k][j] )//����������
                {
                    change = ADC_GetMessage[k][i];
                    ADC_GetMessage[k][i] = ADC_GetMessage[k][j];
                    ADC_GetMessage[k][j] = change;
                }
            }
    
    for(i = Min_SamplingNum;i < SamplingNum - Min_SamplingNum; i++)//������
        for(k = 0;k < 4; k++)
        {
            SUM_ADC_GetMessage[k] += ADC_GetMessage[k][i];
        }
    
    for(k = 0;k < 4; k++)//ȡƽ��
    {
        ADC_Value[k] = SUM_ADC_GetMessage[k] / (SamplingNum - 2 * Min_SamplingNum);
    }
    
    for(k = 0;k < 4; k++)//�����͵����飬�Ա���һ��ʹ��
    {
        SUM_ADC_GetMessage[k] = 0;
    }
    
}

/*******************************************************************************
 *  @brief      ADCnormal����
 *  @note       ��һ����������ÿ����г��Ե�е����ֵ
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCnormal(void)
{ 
  for(k = 0;k < 4; k++)
    {
        ADC_Normal[k] = (float)ADC_Value[k] / (float)ADC_Maxing[k];
    }
  if ( ADC_Normal[1] < 0.001) ADC_Normal[1] = 0.001;
  if ( ADC_Normal[2] < 0.001) ADC_Normal[2] = 0.001;
  if ( ADC_Normal[3] < 0.001) ADC_Normal[3] = 0.001;
  if ( ADC_Normal[0] < 0.001) ADC_Normal[0] = 0.001;
}


/*******************************************************************************
 *  @brief      ADCerror_diff����
 *  @note       ƫ�������
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCerror_diff(void)
{
    /*error_diff_last = error_diff;   
    error_diff = ADC_Normal[2] - ADC_Normal[1];
    if(error_diff >= 0)
    {
        fe = error_diff;
    }
    else
    {
        fe = -error_diff;
    }
    error_diff_c = error_diff - error_diff_last;
    if(error_diff_c >= 0)
    {
        fec = error_diff_c;
    }
    else
    {
        fec = -error_diff_c;
    }*/
  
      fe_last = fe;  //��¼��һ�ε�ֵ  (ADC_Normal[0] * ADC_Normal[0])
    //  fe = (ADC_Normal[2] - ADC_Normal[1]) * 100;  //ֱ�����������ƫ��Ŵ�100��  (ADC_Normal[3] * ADC_Normal[3])
      fe1 =  sqrt(  ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
      fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
      fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
       if( fe < -100) fe = -100;
       if( fe > 100 ) fe = 100;
      fec = fe - fe_last;  //����仯��
      
   // fe = 0.65 * (ADC_Normal[2] - ADC_Normal[1]) + 0.35 * (ADC_Normal[3] - ADC_Normal[0]);
}


/*******************************************************************************
 *  @brief      road_check����
 *  @note       ��ȡ�����ֵ����
                
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void road_check(void)
{
    D_power = 1;
    if( (cross_pass > 1) || ( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ((ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5)) ) )
    {
        if( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ( (ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5) ))   //�жϻ���
        {
            speed_power = 0.1;
            P_power = 2;
            cross = cross + 2;
            if( ADC_Normal[3] > ADC_Normal[0] )  
                cross_left++;
            if( ADC_Normal[0] > ADC_Normal[3] ) 
                cross_right++;
            if( (cross > 250) && (cross_pass < 50) )  
                cross_pass = cross_pass + 100;          
        }
        if( (cross_pass > 1) )
        {
            speed_power = 0.1;
            level = 31;
            if(cross > 0) cross--;
            if(cross_pass > 0) cross_pass--;
            if( cross_left > cross_right )  
            {
                fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe
                fe2 =  sqrt( ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
               // none_steerctrl = 1; steerctrl =  Maxsteering;
            }//��ת
            else                           
            {
                //none_steerctrl = 1; steerctrl = Minsteering; 
            }//��ת
        }   
    } 
    else 
    {       
            if(cross > 0) cross--;
            if(cross_left > 0) cross_left--;
            if(cross_right > 0) cross_right--;
            none_steerctrl = 0;
            if(ADC_Normal[0] < 0.05 && ADC_Normal[3] < 0.05 )   // ֱ�� 
            {
                if( P_power > 0.5 ) P_power = P_power - 0.8 * dreams;   //���ͼ���Pֵ 
                fe1 =  sqrt( 2 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe
                fe2 =  sqrt( 2 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) )  * 100);
                D_power = 0.4;
                speed_power = 1.2;
                level = 1;
            }
            else
            {
               if( ( ADC_Normal[0] < 0.4 || ADC_Normal[3] < 0.4 ) ) //�������Χ��
               {
                  if( (ADC_Normal[0] > ADC_Normal[3] && ADC_Normal[2] > ADC_Normal[1] && ADC_Normal[0] > 0.05) || ( ADC_Normal[3] > ADC_Normal[0] && ADC_Normal[1] > ADC_Normal[2] && ADC_Normal[3] > 0.05 ) )  //�ж��Ƿ�����
                  {
                        fe = (int)( (sqrt( ADC_Normal[2] * ADC_Normal[2] ) - sqrt( ADC_Normal[1] * ADC_Normal[1] ) ) * 100 ); // ϵ������
                        level = 11;
                        if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                        speed_power = 0.8;
                  }
                  else
                  {      
                        if( ADC_Normal[0] < 0.05 || ADC_Normal[3] < 0.05 )  //�ж��Ƿ���һ��ֱ��ƫС
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2 )  //һ��ֱ��ƫС��һ��ֱ��ƫ��
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 2; 
                            }             
                            else     //һ��ֱ��ƫС��һ��ֱ��ƫ�� 
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe *0.5��ʹ�����
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                        }
                        else 
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2)   //��ж�����С ����һ��ƫ��
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] +  ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe  *0.5��ʹ�����
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] +  ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                            
                            else     //�������ƫ��
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 2;
                            }
                        }
                  }
               }
                
            }
            if(ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.4 )   //�ж�ʮ��
            {
                if( P_power < 1.5 ) P_power = P_power + dreams;   //��������Pֵ
                D_power = 0.6;
                speed_power = 0.5;
                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] );  //���¼���fe
                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                level = 21;
            }

    }
    
    if(fe > 100) fe = 100; //����
    if(fe < -100) fe = -100; //����
}
/*******************************************************************************
 *  @brief      Road_Id_Get����
 *  @note       ƫ�������
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void Road_Id_Get()
{
   /*****  Part 1 �����ж� *****/
        if( ((ADC_Normal[0] <= 0.500) || (ADC_Normal[1] <= 0.500)) && ((ADC_Normal[0] >= 0.020) || (ADC_Normal[1] >= 0.020)) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //����ұ�������о�ƫ�����ƫС����Ƕȴ���
        {                                                                                                        //  ����
            steerctrl = Minsteering;//last_steerctrl; //����������С������ƫ
           // beep_on(); //����������������һ��
           // DELAY_MS(20);
           // beep_off();
        }
        
        if( ((ADC_Normal[2] <= 0.500) || (ADC_Normal[3] <= 0.500)) && ((ADC_Normal[2] >= 0.020) || (ADC_Normal[3] >= 0.020)) && (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) )//������������о�ƫ���ұ�ƫС����Ƕȴ���
        {                                                                                                        //  ����
             
            steerctrl = Maxsteering;//last_steerctrl; //���������������ƫ
          //  beep_on(); //����������������һ��
          //  DELAY_MS(20);
          //  beep_off();
        }
                if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //����ĸ���ж�ƫС�������ǳ�ʼ״̬����flag���1��Ȼ������������ѭ��
        {                                                                                                 
          jishu++;
          steerctrl = last_steerctrl;
        }
   
         if(jishu >= 10) //��ʮ����󣬻�����ôС��flag=1���������ͣ��
        {
          flag = 1;
         //speed_forecast = 0;                  
        }
        
        if(( (ADC_Normal[0] >= 0.005) || (ADC_Normal[1] >= 0.005) || (ADC_Normal[2] >= 0.005) || (ADC_Normal[3] >= 0.005))&&(jishu < 10) ) //���ֵ�ָ�����jishu��գ�����������
        {                                                                                                 
          jishu = 0;
        }        
          
        /*****  Part 2  ·��ʶ��*****/
        
///////////////////////////////////////////���������뻷////////////////////////////////
       

          
     
        
        
        /*****  Part 3 ͣ���ж�  *****/
       /* if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
        {                                                                                                 
          flag = 1;                                                                                       
        }*/     
        if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
        {
         speed_forecast = 0;
         
         //steerctrl = 768;
         
           // ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������ 0
            //ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������ 0
       //     LED_PrintShort(50,7,speed_forecast); //��ʾ���PWM
        //  LED_PrintValueF(50,5,speed_fec_max,2); //��ʾ������仯�ʵľ���ֵ
        //     LED_PrintValueF(50,3,speed_min,2); //��ʾ��С�ٶ�
        //     DELAY_MS(100);
             
        }
}

/*******************************************************************************
 *  @brief      Round_about����
 *  @note       ���뻷������
                
 *  @warning    
 ******************************************************************************/
void Round_about() 
{
    ///////////////////////////////////////////���������뵺////////////////////////////////
       
        if((rhd_n_flag == 0)&&(ADC_Normal[1] >= 0.500)&&(ADC_Normal[2] >= 0.900))  //���������ж�
        {
             rhd_n_flag_a = 1;                                                     //���뻷������
             rhd_n_flag = 1;
        }
        if(rhd_n_flag_a == 1)
        {
             if(ADC_Normal[3] >= 0.200)                                            //����·���ж�
             {
                  rhd_n_flag_a = 0; 
                  rhd_n_flag_b = 1;                                                //���ﻷ��·��
                  beep_on();
             }
        }
        if(rhd_n_flag_b==1)       
        {
              speed_power = 0.8;                                                   //�뵺ǰ����
              if(ADC_Normal[3] <= 0.100)                                           //������е͹��ж�
              {
                 rhd_n_flag_b = 0;  
                 rhd_n_flag_c = 1;                                                 //���������
                 beep_off();
               } 
        }     
        if(rhd_n_flag_c == 1)   
        {
              speed_power = 0.8;
              if(ADC_Normal[3] >= 0.100)                                           //������е͹��ж�
              {
                rhd_n_flag_c = 0;  
                rhd_n_flag_d = 1;                                                  //�뵺��
              }     
        }
        if(rhd_n_flag_d == 1)
        {
              speed_power = 0.1;                                                   //�뵺����   
              beep_on();
            
              //////////������㷨����/////////������ɽ��� 
              fe_last = fe;                //�뵺����㷨
              fe = (int)(( (sqrt(ADC_Normal[2]) - sqrt(ADC_Normal[3])) / ( ADC_Normal[2] + ADC_Normal[3] ) ) * 100);
              fec = fe - fe_last;                 
              
              if(steerctrl >= Midsteering + 120)  
                 rhd_n_flag_e = 1;
              if((rhd_n_flag_e == 1)&&(steerctrl <= Midsteering + 60))
              {
                  none_steerctrl = 1;
                  steerctrl = Midsteering + 150;
              }
              //������̫С  
                
              if(ADC_Normal[3] >= 0.200)  
                rhd_n_flag_f = 1;              
              if((rhd_n_flag_f == 1)&&(ADC_Normal[1] >= 0.800))
                rhd_n_flag_g = 1;
              //������������뵺    
              
              if((rhd_n_flag_g == 1)&&((ADC_Normal[1] < 0.600)||(ADC_Normal[2] < 0.600)))   //�ɹ��뵺
              {
                 rhd_n_flag_a = 0;
                 rhd_n_flag_b = 0;
                 rhd_n_flag_c = 0;
                 rhd_n_flag_d = 0;
                 rhd_n_flag_e = 0;
                 rhd_n_flag_f = 0;
                 rhd_n_flag_g = 0;
                 
                 none_steerctrl = 0;
                 speed_power = 1.0; 
                 beep_off();
              } 
             
              
              ///////////////ֱ�Ӵ����ǲ���///////(����󻷻�����)
           /*   none_steerctrl = 1;
              steerctrl = Maxsteering ;
              
              
              if(ADC_Normal[1] >= 0.800)
                rhd_n_flag_e = 1;
               if((rhd_n_flag_e == 1)&&(ADC_Normal[1] < 0.600)||(ADC_Normal[2] < 0.600))
              {
                rhd_n_flag_d = 0;
                rhd_n_flag_e = 0;

                none_steerctrl = 0;
                 speed_power = 1.0; 
                 beep_off();
              }
            */  
        }  
           
 ///////////////////////////////////////////����˳���뵺////////////////////////////////
       
        if((rhd_s_flag == 0)&&(ADC_Normal[2] >= 0.500)&&(ADC_Normal[1] >= 0.900))  //���������ж�
        {
           rhd_s_flag = 1;  
           rhd_s_flag_a = 1;                                                       //���뻷������
        }
        if(rhd_s_flag_a == 1)
        {
             if(ADC_Normal[0] >= 0.200)                                            //����·���ж�
             {
                  rhd_s_flag_a = 0; 
                  rhd_s_flag_b = 1;                                                //���ﻷ��·��
                  beep_on();
             }
        }
        if(rhd_s_flag_b==1)       
        {
              speed_power = 0.8;                                                   //�뵺ǰ����
              if(ADC_Normal[0] <= 0.100)                                          //������е͹��ж�
              {
                 rhd_s_flag_b = 0;  
                 rhd_s_flag_c = 1;                                                 //���������
                 beep_off();
               } 
        }     
        if(rhd_s_flag_c == 1)   
        {
              speed_power = 0.8;
              if(ADC_Normal[0] >= 0.100)                                          //������е͹��ж�
              {
                rhd_s_flag_c = 0;  
                rhd_s_flag_d = 1;                                                   //�뵺��
              }     
        }
        if(rhd_s_flag_d == 1)
        {
              speed_power = 0.1;                                                   //�뵺����   
               beep_on();         
              
              //////////������㷨����///////// ��˳��򲻽�ȥ��
      /*        fe_last = fe;                //�뵺����㷨
              fe = (int)(( (sqrt(ADC_Normal[0]) - sqrt(ADC_Normal[1])) / ( ADC_Normal[1] + ADC_Normal[0] ) ) * 100);
              fec = fe - fe_last;                 
              
              if(steerctrl <= Midsteering - 120)  //������̫С
                 rhd_s_flag_e = 1;
              if((rhd_s_flag_e == 1)&&(steerctrl >= Midsteering - 60))
              {
                  none_steerctrl = 1;
                  steerctrl = Midsteering - 150;
              }
                
                
              if(ADC_Normal[0] >= 0.100)  
                rhd_s_flag_f = 1;              
              if((rhd_s_flag_f == 1)&&(ADC_Normal[2] >= 0.800))
                rhd_s_flag_g = 1;
              //������������뵺    
              
              if((rhd_s_flag_g == 1)&&((ADC_Normal[1] < 0.600)||(ADC_Normal[2] < 0.600)))   //�ɹ��뵺
              {
                 rhd_s_flag_a = 0;
                 rhd_s_flag_b = 0;
                 rhd_s_flag_c = 0;
                 rhd_s_flag_d = 0;
                 rhd_s_flag_e = 0;
                 rhd_s_flag_f = 0;
                 rhd_s_flag_g = 0;
                 
                 none_steerctrl = 0;
                 speed_power = 1.0; 
                 beep_off();
              } 
          */   
              
              ///////////////ֱ�Ӵ����ǲ���///////(���Խ�ȥ)
              none_steerctrl = 1;
              steerctrl = Minsteering ;
              
              
              if(ADC_Normal[2] >= 0.800)
                rhd_s_flag_e = 1;
               if((rhd_s_flag_e == 1)&&(ADC_Normal[1] < 0.600)||(ADC_Normal[2] < 0.600))
              {
                rhd_s_flag_d = 0;
                rhd_s_flag_e = 0;

                none_steerctrl = 0;
                 speed_power = 1.0; 
                 beep_off();
              }   
        }  
          
  

        
/////////////////////////////�������//////////            
        if((rhd_n_flag == 1)&&(ADC_Normal[3] >= 0.700)&&((ADC_Normal[1]-ADC_Normal[0])*(ADC_Normal[1]-ADC_Normal[0]) <= 0.250))
        {
          chd_n_flag_a = 1;
          beep_on();
        }
        if(chd_n_flag_a == 1)                      //����
        {
            speed_power = 0.8;                    //��������   
              
            fe_last = fe;                        //��������㷨
            //fe = (int)(( (sqrt(ADC_Normal[3]) - sqrt(ADC_Normal[0])) / ( ADC_Normal[3] + ADC_Normal[0] ) ) * 100);
            fe = (int)( (ADC_Normal[3] - ADC_Normal[0] ) * 100);
            fec = fe - fe_last;     
          
          if(ADC_Normal[3] <= 0.700)               
          {
             chd_n_flag_a = 0;
             chd_n_flag = 1;                      //��������
             
             beep_off();
             speed_power = 1.0;                    //�ͷż���
          }
          if((chd_n_flag == 1)&&(ADC_Normal[0] <= 0.050)&&(ADC_Normal[3] <= 0.050))   //�뿪��������
          {
              rhd_n_flag_a = 0;
              rhd_n_flag_b = 0;
              rhd_n_flag_c = 0;
              rhd_n_flag_d = 0;
              rhd_n_flag_e = 0;
              rhd_n_flag_f = 0;
              rhd_n_flag_g = 0;
              chd_n_flag_a = 0;
            
              rhd_n_flag = 0;
              chd_n_flag = 0;                                                      //�����б�־λ
          }

        } 
           
////////////////////////////˳�����//////////            
        if((rhd_s_flag == 1)&&(ADC_Normal[0] >= 0.700)&&((ADC_Normal[2]-ADC_Normal[3])*(ADC_Normal[2]-ADC_Normal[3]) <= 0.250))
        {
          chd_s_flag_a = 1;
          beep_on();
        }
        if(chd_s_flag_a == 1)                      //����
        {
        
              
            speed_power = 0.8;                     //��������   
               
            fe_last = fe;                         //��������㷨
            //fe = (int)(( (sqrt(ADC_Normal[3]) - sqrt(ADC_Normal[0])) / ( ADC_Normal[3] + ADC_Normal[0] ) ) * 100);
            fe = (int)( (ADC_Normal[3] - ADC_Normal[0] ) * 100);
            fec = fe - fe_last;     
          
          if(ADC_Normal[0] <= 0.700)               
          {
             chd_s_flag_a = 0;
             chd_s_flag = 1;                        //��������
             beep_off();
             speed_power = 1.0;                     //�ͷż���
          }
          if((chd_n_flag == 1)&&(ADC_Normal[0] <= 0.050)&&(ADC_Normal[3] <= 0.050))   //�뿪��������
          {
              rhd_s_flag_a = 0;
              rhd_s_flag_b = 0;
              rhd_s_flag_c = 0;
              rhd_s_flag_d = 0;
              rhd_s_flag_e = 0;
              rhd_s_flag_f = 0;
              rhd_s_flag_g = 0;
              chd_s_flag_a = 0;
            
              rhd_s_flag = 0;
              chd_s_flag = 0;                                                          //�����б�־λ
          }  
          
         }
}   

/*******************************************************************************
 *  @brief      Cross_road ����
 *  @note       ʮ��·���ж�
                
 *  @warning    
 ******************************************************************************/
uint8 Cross_road()
{
  if((ADC_Normal[0] >= 0.750)&&(ADC_Normal[3] >= 0.750))    
    return 1 ;
  else
    return 0;
}