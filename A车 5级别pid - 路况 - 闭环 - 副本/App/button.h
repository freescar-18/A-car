/*!
 * @file       button.h
 * @brief      �жϷ�����
 * @author     
 * @version    A��
 * @date       
 */

#ifndef __BUTTON_H__
#define __BUTTON_H__


/**************************  ����������  **************************************/
void PORTE_IRQHandler(void);  //PORTE�Ĳο��жϷ�����
void PORTC_IRQHandler(void);
void PORTB_IRQHandler(void);
void PORTA_IRQHandler(void);
void switch_read(void);

/****************************  �궨��  ****************************************/

#endif  //__BUTTON_H__