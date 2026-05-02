#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

#define KEY_S PAin(5)
#define KEY_P PCin(15)
#define KEY_J PAin(6)
#define KEY_M PCin(14)
#define KEY_X PCin(13)

void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);           //����ɨ��  
#endif  
