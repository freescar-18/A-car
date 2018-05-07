/*!
 * @file       AngleControl.c
 * @brief      角度控制函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
int8 l;
int8 pe,pec,num; 
int16 steerctrl; // 输出的舵机转角PWM
int16 last_steerctrl; // 上次输出的舵机转角PWM
int16 steerctrl_error; //舵机转角的增量（可正可负） 加上舵机中间值即为舵机需要转角的PWM

float steer_P,steer_D;  // 输出的 P 值 和 D 值
extern float fe,fec,fe_last; 
float eFuzzy[2] = {0,0}; 
float ecFuzzy[2] = {0,0}; 
float Fuzzy_kp[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};
float Fuzzy_kd[6] = {0,0,0,0,0,/*末尾为0,用来查询模糊表步骤*/ 0};

/*需要调节的参数*/
float P_power = 1;
float D_power = 1;
float eRule[5] = {-50,-25,0,25,50}; //输入误差（fe）的范围，由负到正 
                                            //如归一化偏差法，输入的误差为-0.9 到 0.9，乘以100即为-90到90，再分成7份
float ecRule[5] = {-25,-15,0,15,25}; //输入误差的变化率（fec）的范围，由负到正
float Rule_kp[5] = {-0.7,-0.35,0,0.4,0.8};  //  输出的P值的范围  
                                                   //  负即是左转的P值（left -- right）(+85 - -75)
float Rule_kd[5] = {-5,-2.5,0,2.5,5};  //输出的D值的范围
/*
int rule_kp[8][8]=  //p值规则表
{
  //ec 0 1 2 3 4 5 6  //e
      {6,6,5,5,4,3,3,7},//0    
      {6,6,5,4,4,3,2,7},//1   
      {6,5,5,4,3,2,2,7},//2    
      {5,5,4,3,2,1,1,7},//3         
      {4,4,3,2,2,1,0,7},//4
      {4,3,2,1,1,1,0,7},//5
      {3,3,2,2,2,0,0,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
int rule_kp[6][6]=  //p值规则表
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0    
      {4,3,3,2,1,5},//1   
      {4,3,2,1,0,5},//2    
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4
      {5,5,5,5,5,5} //
};
/*
int rule_kd[8][8]=  //d值规则表
{
  //ec 0 1 2 3 4 5 6  //e
      {0,0,1,3,5,6,6,7},//0    
      {0,1,2,3,4,5,6,7},//1   
      {1,2,2,3,4,4,5,7},//2   
      {3,3,3,3,3,3,3,7},//3    
      {5,4,4,3,2,2,1,7},//4
      {6,5,4,3,2,1,0,7},//5
      {6,6,5,3,1,0,0,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
/*
int rule_kd[8][8]=  //d值规则表
{
  //ec 0 1 2 3 4 5 6  //e
      {6,6,5,3,1,0,0,7},//0    
      {6,5,4,3,2,1,0,7},//1   
      {5,4,4,3,2,2,1,7},//2   
      {3,3,3,3,3,3,3,7},//3    
      {1,2,2,3,4,4,5,7},//4
      {0,1,2,3,4,5,6,7},//5
      {0,0,1,3,5,6,6,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
/*
int rule_kd[8][8]=  //d值规则表
{
  //ec 0 1 2 3 4 5 6  //e
      {0,0,1,2,2,3,3,7},//0    
      {0,1,1,1,2,3,4,7},//1   
      {0,1,2,2,3,4,4,7},//2   
      {1,2,2,3,4,4,5,7},//3    
      {2,2,3,4,4,5,6,7},//4
      {2,3,4,5,5,5,6,7},//5
      {3,3,4,4,5,6,6,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
int rule_kd[6][6]=  //p值规则表
{
  //ec 0 1 2 3 4  //e
      {3,1,0,0,3,5},//0    
      {2,1,1,1,2,5},//1   
      {2,1,1,1,2,5},//2    
      {3,2,2,2,3,5},//3         
      {4,3,3,3,4,5},//4
      {5,5,5,5,5,5} //
};




/*******************************************************************************
 *  @brief      fuzzy_mem_cal函数
 *  @note       隶属度计算函数
                输出结果放在eFuzzy[]和ecfuzzy[]，等级为pe、pec
 *  @warning    18/3/10 代码参考binary-star队  v3.0
 ******************************************************************************/
void fuzzy_mem_cal(void)//隶属度计算
{
  /*-----误差隶属函数描述-----*/
    if(fe < eRule[0])     //用来确定隶属度		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(fe < eRule[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (eRule[1]-fe)/(eRule[1]-eRule[0]);
      pe = 0;
    }
    else if(fe < eRule[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (eRule[2] -fe)/(eRule[2]-eRule[1]);
      pe =1;
    }
    else if(fe < eRule[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (eRule[3] -fe)/(eRule[3]-eRule[2]);
      pe =2;
    }
    else if(fe < eRule[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (eRule[4]-fe)/(eRule[4]-eRule[3]);
      pe=3;
    }	
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
    
    /*-----误差变化隶属函数描述-----*/
    if(fec <= ecRule[0])
    {
      ecFuzzy[0] =1.0;
      pec = 0;
    }
    else if(fec < ecRule[1])
    {
      ecFuzzy[0] = (ecRule[1] - fec)/(ecRule[1]-ecRule[0]);
      pec = 0 ;
    }
    else if(fec < ecRule[2])
    {
      ecFuzzy[0] = (ecRule[2] - fec)/(ecRule[2]-ecRule[1]);
      pec = 1;
    }
    else if(fec < ecRule[3])
    {
      ecFuzzy[0] = (ecRule[3] - fec)/(ecRule[3]-ecRule[2]);
      pec = 2 ;
    }
    else if(fec < ecRule[4])
    { 
      ecFuzzy[0] = (ecRule[4] - fec)/(ecRule[4]-ecRule[3]);
      pec=3;
    }	
    else										
    {
      ecFuzzy[0] =1.0;
      pec=4;
    }
    ecFuzzy[1] = 1.0 - ecFuzzy[0];
}

/*******************************************************************************
 *  @brief      fuzzy_query函数
 *  @note       查询模糊规则表，算出输出的各占比
                
 *  @warning    18/3/10 代码参考binary-star队  v3.0
 ******************************************************************************/
void fuzzy_query(void)//查询模糊规则表
{
    for(l = 0;l <= rank; l++) //清空数组以便累加
    {
        Fuzzy_kp[l] = 0;
        Fuzzy_kd[l] = 0;
    }
      /*查询kp模糊规则表*/  
    num = rule_kp[pe][pec];
    Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = rule_kp[pe][pec+1];
    Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = rule_kp[pe+1][pec];
    Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = rule_kp[pe+1][pec+1];
    Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];
    
      /*查询kd模糊规则表*/  
    num = rule_kd[pe][pec];
    Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[0];

    num = rule_kd[pe][pec+1];
    Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = rule_kd[pe+1][pec];
    Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = rule_kd[pe+1][pec+1];
    Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      fuzzy_solve函数
 *  @note       重心法解模糊
                
 *  @warning    18/3/10 代码参考binary-star队 v3.0
 ******************************************************************************/
void fuzzy_solve(void)//解模糊得到pd值
{
    steer_P = 0; //清空P和D值以便累加
    steer_D = 0;
    /*面积中心法解模糊*/
    for(l = 0;l < rank; l++)
    {
      steer_P += Fuzzy_kp[l] * Rule_kp[l];
      steer_D += Fuzzy_kd[l] * Rule_kd[l];   
    }
}

/*******************************************************************************
 *  @brief      steercontrol函数
 *  @note       舵机转角函数
                
 *  @warning    18/3/10 v3.0
 ******************************************************************************/
void steercontrol(void) 
{
    if(steer_P < 0)  steer_P = -steer_P; //将输出的 P 值变为正数，乘上输入的误差恰好为舵机转角的增量
    if(steer_D < 0)  steer_D = -steer_D; //将输出的 D 值变为正数
    steerctrl_error = (int)( P_power * steer_P * fe + D_power * steer_D * (fe - fe_last) );//舵机转角增量
    last_steerctrl = steerctrl;
    steerctrl = Midsteering + steerctrl_error; //舵机转角PWM
}
    


    