/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"

/*!
 *  @brief      UART3中断服务函数
 *  @since      v5.0
 */


unsigned char Re_buf[11],counter;
unsigned char ucStra[6],ucStrw[6],ucStrAngle[6];

float d_error,last;
uint8 flag,flag_run;
uint8 i;
uint8 flag_pit=0;
int32 PWM1,PWM2,PWM3,PWM4;
float Value[3];
uint8 flag_pingh=0;
#define dianji1_CH    FTM_CH4
#define dianji2_CH    FTM_CH5
#define dianji3_CH    FTM_CH0
#define dianji4_CH    FTM_CH1

void PIT0_IRQHandler();

void uart5_handler(void)
{
    char ch;

    if(uart_query    (UART5) == 1)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar   (UART5, &ch);                    //无限等待接受1个字节
        Re_buf[counter]=ch;	
	
	    if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头
	              
	    counter++; 
	    if(counter==11)             //接收到11个数据
	    {    
	       counter=0;               //重新赋值，准备下一帧数据的接收        
			switch(Re_buf [1])
			{
			case 0x51:
			ucStra[0]=Re_buf[2];
			ucStra[1]=Re_buf[3];
			ucStra[2]=Re_buf[4];
			ucStra[3]=Re_buf[5];
			ucStra[4]=Re_buf[6];
			ucStra[5]=Re_buf[7];
			break;
			case 0x52:	 
			ucStrw[0]=Re_buf[2];
			ucStrw[1]=Re_buf[3];
			ucStrw[2]=Re_buf[4];
			ucStrw[3]=Re_buf[5];
			ucStrw[4]=Re_buf[6];
			ucStrw[5]=Re_buf[7];
			break;
			case 0x53: 
			ucStrAngle[0]=Re_buf[2];
			ucStrAngle[1]=Re_buf[3];
			ucStrAngle[2]=Re_buf[4];
			ucStrAngle[3]=Re_buf[5];
			ucStrAngle[4]=Re_buf[6];
			ucStrAngle[5]=Re_buf[7];	
			break;
			} 
                        Value[0] = ((short)(ucStra[1]<<8| ucStra[0]))/32768.0*16;
			Value[1] = ((short)(ucStra[3]<<8| ucStra[2]))/32768.0*16;
			Value[2] = ((short)(ucStra[5]<<8| ucStra[4]))/32768.0*16;
			Oprintfloat(0,0,Value[0]);
			Oprintfloat(0,1,Value[1]);
                        Oprintfloat(0,2,Value[2]);
		 	Value[0] = ((short)(ucStrw[1]<<8| ucStrw[0]))/32768.0*2000;
			Value[1] = ((short)(ucStrw[3]<<8| ucStrw[2]))/32768.0*2000;
			Value[2] = ((short)(ucStrw[5]<<8| ucStrw[4]))/32768.0*2000;
			Oprintfloat(0,3,Value[0]); 
			Oprintfloat(0,4,Value[1]);
                        
		 	Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
			Value[1] = ((short)(ucStrAngle[3]<<8| ucStrAngle[2]))/32768.0*180;
			Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
			Oprintfloat(0,5,Value[0]); 
			Oprintfloat(0,6,Value[1]);
                        Oprintfloat(0,7,Value[2]);
                  
                   if(flag_run==1)
                   {
                    
                    d_error= Value[0]-last;
                    last=Value[0];
                    pit_init_ms(PIT0,200);
                    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);
                    enable_irq(PIT0_IRQn);
                    flag_run=0;
                    //gpio_turn(PTD0);
                   }
                   if(flag_pingh==0)
                   {
                    PWM1=(int32)Value[0]*800;
                    PWM3=(int32)Value[0]*800;
                    
                     if(PWM1<0)
                  {
                    if(PWM1<-2300)
                    {
                      PWM1=-2300;
                    }
                    PWM2=PWM1*(-1);
                    PWM1=0;
                    
                  }
                  else
                  {
                    if(PWM1>2300)
                    {
                      PWM1=2300;
                    }
                    PWM2=0;
                  }
     if(PWM3<0)
                  {
                    if(PWM3<-2300)
                    {
                      PWM3=-2300;
                    }
                    PWM4=PWM3*(-1);
                    PWM3=0;
                    
                  }
                  else
                  {
                    if(PWM3>2300)
                      PWM3=2300;
                    PWM4=0;
                  }
                    
                    
                   }
                   

                  
                  
                  
                   
                  
                  
                  if(d_error<-0.5)
                  {
                    flag_pingh=1;
                   
                     PWM2=(int32)(3000-(5-Value[0])*100);
                     PWM4=(int32)(3000-(5-Value[0])*100);
                     if(PWM2>2800)
                     {
                       PWM2=2800;
                     }
                     if(PWM2<0)
                     {
                       PWM2=0;
                     }
                     if(PWM4>2800)
                     {
                       PWM4=2800;
                     }
                     if(PWM4<0)
                     {
                       PWM4=0;
                     }
                     PWM1=0;
                     PWM3=0;
                      d_error=0;
                  }
                  else if(d_error>0.5)
                  {
                    flag_pingh=1;
                    
                    if(flag_pit==0)
                    {
                      //enable_irq(PIT0_IRQn);
                      flag_pit=1;
                    }
                   
                    PWM1=(int32)(3000-(Value[0]+5)*100);
                    PWM3=(int32)(3000-(Value[0]+5)*100);
                     if(PWM1>2800)
                     {
                       PWM1=2800;
                     }
                     if(PWM1<0)
                     {
                       PWM1=0;
                     }
                     if(PWM3>2800)
                     {
                       PWM3=2800;
                     }
                     if(PWM3<0)
                     {
                       PWM3=0;
                     }
                     PWM2=0;
                     PWM4=0;
                     d_error=0;
                  }
                     ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
                          ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
                          ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
                          ftm_pwm_duty(FTM1, dianji4_CH,PWM4);
                  
                  
	      }                  //发送字符串
    }
}

/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       串口中断接收测试
 */
void main()
{
  
  uint8 flag_begin,flag_temp;   
  
  unsigned char i=0;
  
    ftm_pwm_init(FTM0, dianji1_CH,1000,0);
    ftm_pwm_init(FTM0, dianji2_CH,1000,0);
    ftm_pwm_init(FTM1, dianji3_CH,1000,0);
    ftm_pwm_init(FTM1, dianji4_CH,1000,0);
    gpio_init (PTD0, GPO,0);//beep bingo
     // 设置中断服务函数到中断向量表里
    flag_run=1;
    //uart_init(UART3,115200);     //初始化串口(UART3 是工程里配置为printf函数输出端口，故已经进行初始化)
 set_vector_handler(UART5_RX_TX_VECTORn,uart5_handler);
 uart_rx_irq_en (UART5);
  
  while(ucStrAngle[0]==0X00)
    {
      Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
      if(Value[0] < 5 && Value[0] > -5)
        
          flag_begin = 1;//说明一开始在平地上
        
        else 
          flag_begin = 2;//说明一开始在坡道上
    }
              //发送字符串

      // 设置中断服务函数到中断向量表里

                                     //开串口接收中断
    LCD_Init();
     
     
   
    //定时 1000 个bus时钟 后中断
    
    
    
   
              
                       
        flag_begin = 1;
        
        if(flag_begin == 1)//说明一开始在平地上
        {
          flag=1;
          if(flag==0)//第一阶段
          {
            while(Value[0]>=-5&&Value[0]<=5)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=3000;
              PWM2=0;//前转PWM 
              PWM3=3000;
              PWM4=0;//后转PWM
              
              
              PWM3=PWM3 + Value[2]*500;
              
              
              PWM1=PWM1 - Value[2]*500;
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);
            }
            flag=1;
          }
          if(flag==1)//第二阶段
          {
            i=0;
            flag_temp=0;
            while(flag==1)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=350*Value[0]+1000;
              PWM2=0;
              PWM3=350*Value[0]+1000;  
              PWM4=0;
             
              
              PWM3=PWM3 - Value[2]*500;      
              PWM1=PWM1 + Value[2]*500;
               if(PWM1>7500)
              {
                PWM1=7500;
              }              
              if(PWM3>7500)
              {
                PWM3=7500;
              }
              if(PWM1<0)
              {
                PWM1=0;
              }              
              if(PWM3<0)
              {
                PWM3=0;
              }
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);
              if(Value[0]>=-5.0&&Value[0]<=5.0&&flag_temp==0) 
              {
                 while(1)
                 {
                  Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
                  Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;                        
                 }
                
                flag_temp=1;  
              }
            }            
          }
          if(flag==2)//第三个阶段
          {
            
            while(Value[0] > -10)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=7000+350*Value[0];
              PWM2=0;
              PWM3=7000+Value[0]+2500;  
              PWM4=0;
             
              
              PWM3=PWM3 - Value[2]*500;      
              PWM1=PWM1 + Value[2]*500;
               if(PWM1>7500)
              {
                PWM1=7500;
              }              
              if(PWM3>7500)
              {
                PWM3=7500;
              }
              if(PWM1<0)
              {
                PWM1=0;
              }              
              if(PWM3<0)
              {
                PWM3=0;
              }
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);    
            }
            flag=3;
          }
          if(flag == 3)
          {
            //现在要退回家去
            while(Value[0] < 10)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              
              PWM1=0;
              PWM2=7000;
              PWM3=0;  
              PWM4=7000;
             
              
              PWM4=PWM4 + Value[2]*500;      
              PWM2=PWM2 - Value[2]*500;
               if(PWM4>7500)
              {
                PWM4=7500;
              }              
              if(PWM2>7500)
              {
                PWM2=7500;
              }
              if(PWM4<0)
              {
                PWM4=0;
              }              
              if(PWM2<0)
              {
                PWM2=0;
              }
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);    
            }
            ftm_pwm_duty(FTM0, dianji1_CH,0);
            ftm_pwm_duty(FTM0, dianji2_CH,0);
            ftm_pwm_duty(FTM1, dianji3_CH,0);
            ftm_pwm_duty(FTM1, dianji4_CH,0); 
          }
      }
        else//说明一开始在坡道上
        {
          flag=1;
           if(flag==1)//第二阶段
          {
            flag_temp=0;
            while(flag==1)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=350*Value[0]+500;
              PWM2=0;
              PWM3=350*Value[0]+0;  
              PWM4=0;
             
              
              PWM3=PWM3 - Value[2]*500;      
              PWM1=PWM1 + Value[2]*500;
               if(PWM1>7500)
              {
                PWM1=7500;
              }              
              if(PWM3>7500)
              {
                PWM3=7500;
              }
              if(PWM1<0)
              {
                PWM1=0;
              }              
              if(PWM3<0)
              {
                PWM3=0;
              }
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);
              if(Value[0]>=-5&&Value[0]<=5)
                
              {
                while(1)
                {
                  Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=400*Value[0];
              PWM2=0;
              PWM3=400*Value[0];  
              PWM4=0;
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);
                }
                flag_temp=1;
                enable_irq(PIT0_IRQn); 
              }
            }            
          }
          if(flag==2)//第三个阶段
          {
  
            while(Value[0] > -10)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=7000+350*Value[0];
              PWM2=0;
              PWM3=7000+Value[0]+2500;  
              PWM4=0;
             
              
              PWM3=PWM3 - Value[2]*500;      
              PWM1=PWM1 + Value[2]*500;
               if(PWM1>7500)
              {
                PWM1=7500;
              }              
              if(PWM3>7500)
              {
                PWM3=7500;
              }
              if(PWM1<0)
              {
                PWM1=0;
              }              
              if(PWM3<0)
              {
                PWM3=0;
              }
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);    
            }
            flag=3;
          }
          if(flag == 3)
          {
            //现在要退回家去
            while(Value[0] < 10)
            {
              Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
              Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
              PWM1=0;
              PWM2=7000;
              PWM3=0;  
              PWM4=7000;
             
              
              PWM4=PWM4 + Value[2]*500;      
              PWM2=PWM2 - Value[2]*500;
               if(PWM4>7500)
              {
                PWM4=7500;
              }              
              if(PWM2>7500)
              {
                PWM2=7500;
              }
              if(PWM4<0)
              {
                PWM4=0;
              }              
              if(PWM2<0)
              {
                PWM2=0;
              }
              ftm_pwm_duty(FTM0, dianji1_CH,PWM1);
              ftm_pwm_duty(FTM0, dianji2_CH,PWM2);
              ftm_pwm_duty(FTM1, dianji3_CH,PWM3);
              ftm_pwm_duty(FTM1, dianji4_CH,PWM4);    
            }
            ftm_pwm_duty(FTM0, dianji1_CH,0);
            ftm_pwm_duty(FTM0, dianji2_CH,0);
            ftm_pwm_duty(FTM1, dianji3_CH,0);
            ftm_pwm_duty(FTM1, dianji4_CH,0); 
          }
        }
                        
    
}
void PIT0_IRQHandler(void)
{
  flag_run=1;
  pit_close(PIT0);
}
