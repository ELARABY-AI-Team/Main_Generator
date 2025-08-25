/********************************************************************************************************************************************/
/*  Author      : Sherif Elgiar                                                                                                             */
/*  Module      : Component_SSD_C                                                                                                                       */
/*  Last Update : 01 / 04 / 2023                                                                                                            */
/********************************************************************************************************************************************/
/**Includes ==================================================================================*/
#include "Component_SSD.h"
/**Static Variables ==========================================================================*/
static tbyte dig[SSD_MAX];
static tbyte ssd_n;
static tbyte ssd_f,ssd_i,ssd_r;
static tbyte ssd_tick_i;
/**Functions =================================================================================*/
/* @brief :*/
void SSD_Variables_Reset()
{
 ssd_f=0;
 ssd_r=0;
 ssd_i=0;
}
/* @brief :*/
void SSD_Init()
{
 WDT_Reset();
 /*Variables */
 SSD_Variables_Reset();
 ssd_n=0;
 ssd_tick_i=0;
 /* Hardware */
 GPIO_Output_Init(COM0_PORT,COM0_PIN,ssd_off , normal_usage);   
 #ifdef COM1_PORT
 GPIO_Output_Init(COM1_PORT,COM1_PIN,ssd_off ,normal_usage);
 #endif     
 #ifdef COM2_PORT
 GPIO_Output_Init(COM2_PORT,COM2_PIN,ssd_off,normal_usage);
 #endif     
 #ifdef COM3_PORT
 GPIO_Output_Init(COM3_PORT,COM3_PIN,ssd_off,normal_usage);
 #endif 
 #ifdef COM4_PORT
 GPIO_Output_Init(COM4_PORT,COM4_PIN,ssd_off,normal_usage);
 #endif
 #ifdef COM5_PORT
 GPIO_Output_Init(COM5_PORT,COM5_PIN,ssd_off,normal_usage);
 #endif
 #ifdef COM6_PORT
 GPIO_Output_Init(COM6_PORT,COM6_PIN,ssd_off,normal_usage);
 #endif
 #ifdef COM7_PORT
 GPIO_Output_Init(COM7_PORT,COM7_PIN,ssd_off ,normal_usage);
 #endif
 
 GPIO_Output_Init(A_PORT,A_PIN,ssd_off,normal_usage);
 GPIO_Output_Init(B_PORT,B_PIN,ssd_off,normal_usage);
 GPIO_Output_Init(C_PORT,C_PIN,ssd_off,normal_usage);
 GPIO_Output_Init(D_PORT,D_PIN,ssd_off,normal_usage);
 GPIO_Output_Init(E_PORT,E_PIN,ssd_off,normal_usage);
 GPIO_Output_Init(F_PORT,F_PIN,ssd_off,normal_usage);
 GPIO_Output_Init(G_PORT,G_PIN,ssd_off,normal_usage);
}
//--------------------------------------------------------------   
void SSD_Com_Off()
{
 WDT_Reset();
 GPIO_Value_Set(COM0_PORT,COM0_PIN,ssd_off );
 #ifdef COM1_PORT
 GPIO_Value_Set(COM1_PORT,COM1_PIN,ssd_off);
 #endif
 #ifdef COM2_PORT
 GPIO_Value_Set(COM2_PORT,COM2_PIN,ssd_off);
 #endif
 #ifdef COM3_PORT
 GPIO_Value_Set(COM3_PORT,COM3_PIN,ssd_off);
 #endif
 #ifdef COM4_PORT
 GPIO_Value_Set(COM4_PORT,COM4_PIN,ssd_off);
 #endif
 #ifdef COM5_PORT
GPIO_Value_Set(COM5_PORT,COM5_PIN,ssd_off);
 #endif
 #ifdef COM6_PORT
 GPIO_Value_Set(COM6_PORT,COM6_PIN,ssd_off);
 #endif
 #ifdef COM7_PORT
 GPIO_Value_Set(COM7_PORT,COM7_PIN,ssd_off);
 #endif
}
void SSD_Segment_Off()
{
	WDT_Reset();
	/*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_off);
 	/*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_off);
 	/*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_off);
 	/*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_off);
 	/*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_off);
 	/*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_off);
 	/*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_off);
} 
//-------------------------------------------------------------- 
/* @brief :*/
void DIGIT_Set(tbyte no)
 {
 	WDT_Reset();
 	switch(no)
 	    {
 	    	case(0):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     }break;
 	    	case(1):
 	    	         {
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     }break;
 	    	case(2):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case(3):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case(4):
 	    	         {
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case(5):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case(6):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case(7):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     }break;
 	    	case(8):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case(9):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('A'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('b'):
 	    	         {
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('C'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     }break;
 	    	case('d'):
 	    	         {
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('E'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('F'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('G'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     }break;
 	    	case('h'):
 	    	         {
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('i'):
 	    	         {
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     }break;
 	    	case('J'):
 	    	         {
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     }break;
 	    	case('L'):
 	    	         {
 	    		     /*B*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     }break;
 	    	case('N'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     }break;
 	    	case('n'):
 	    	         {
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('o'):
 	    	         {
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('P'):
 	    	         {
 	    		     /*A*/   GPIO_Value_Set(A_PORT,A_PIN,ssd_on);
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('r'):
 	    	         {
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('t'):
 	    	         {
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*G*/   GPIO_Value_Set(G_PORT,G_PIN,ssd_on);
 	    		     }break;
 	    	case('U'):
 	    	         {
 	    		     /*B*/   GPIO_Value_Set(B_PORT,B_PIN,ssd_on);
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     /*F*/   GPIO_Value_Set(F_PORT,F_PIN,ssd_on);
 	    		     }break;
 	    	case('u'):
 	    	         {
 	    		     /*C*/   GPIO_Value_Set(C_PORT,C_PIN,ssd_on);
 	    		     /*D*/   GPIO_Value_Set(D_PORT,D_PIN,ssd_on);
 	    		     /*E*/   GPIO_Value_Set(E_PORT,E_PIN,ssd_on);
 	    		     }break;
 	    }
 }
 //-------------------------------------------------------------- 
 /* @brief :*/
void SSD_8Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4,tbyte d5,tbyte d6,tbyte d7)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==8
    dig[0]=d0;dig[1]=d1;dig[2]=d2;dig[3]=d3;dig[4]=d4;dig[5]=d5;dig[6]=d6;dig[7]=d7;
    #endif
} 
void SSD_7Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4,tbyte d5,tbyte d6)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==7
    dig[0]=d0;dig[1]=d1;dig[2]=d2;dig[3]=d3;dig[4]=d4;dig[5]=d5;dig[6]=d6;
    #endif
}
void SSD_6Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4,tbyte d5)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==6
    dig[0]=d0;dig[1]=d1;dig[2]=d2;dig[3]=d3;dig[4]=d4;dig[5]=d5;
    #endif
}
void SSD_5Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==5
    dig[0]=d0;dig[1]=d1;dig[2]=d2;dig[3]=d3;dig[4]=d4;
    #endif
}
void SSD_4Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==4
    dig[0]=d0;dig[1]=d1;dig[2]=d2;dig[3]=d3;
    #endif
}
void SSD_3Digits_Set(tbyte d0,tbyte d1,tbyte d2)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==3
    dig[0]=d0;dig[1]=d1;dig[2]=d2;
    #endif
}
void SSD_2Digits_Set(tbyte d0,tbyte d1)
{
	WDT_Reset();
    SSD_Variables_Reset();
    #if SSD_MAX==2
    dig[0]=d0;dig[1]=d1;
    #endif
} 

void SSD_1Digits_Set(tbyte d0)
{
	WDT_Reset();
    SSD_Variables_Reset();
    dig[0]=d0;
}
 //-------------------------------------------------------------- 
void SSD_Flash_Set()
{
WDT_Reset();
SSD_Variables_Reset();
 ssd_f=1;	
}
void SSD_Flash_Reset()
{
WDT_Reset();
SSD_Variables_Reset();
 ssd_f=0;	
}  
 //-------------------------------------------------------------- 
 
/* @brief :*/
void SSD_Update(void/*5Msec*/)
{
	    WDT_Reset();
	    if(ssd_f){ssd_i++;if(ssd_i==SSD_FLASH_TIME){ssd_i=0;ssd_r=1-ssd_r;}}
	    SSD_Com_Off();
        SSD_Segment_Off();
        ssd_n++;if(ssd_n>=SSD_MAX){ssd_n=0;}
        if(!ssd_r)
             { 
	         switch(ssd_n)
	                  {
		              case (0):/*COM0*/
		                     {	
		                     DIGIT_Set(dig[0]);
	  	                     GPIO_Value_Set(COM0_PORT,COM0_PIN,ssd_on);	
		                     }break;
		              case (1):/*COM1*/
		                     {
		                     #ifdef COM1_PORT
		                     DIGIT_Set(dig[1]);
		                     GPIO_Value_Set(COM1_PORT,COM1_PIN,ssd_on);	
		                     #endif	
		                     }break;
		              case (2):/*COM2*/
		                     {
		                     #ifdef COM2_PORT
		                     DIGIT_Set(dig[2]);
		                     GPIO_Value_Set(COM2_PORT,COM2_PIN,ssd_on);	
		                     #endif	
		                     }break;
		              case (3):/*COM3*/
		                     {
		                     #ifdef COM3_PORT
		                     DIGIT_Set(dig[3]);
		                     GPIO_Value_Set(COM3_PORT,COM3_PIN,ssd_on);	
		                     #endif	
		                     }break;
		              case (4):/*COM4*/
		                     {
		                     #ifdef COM4_PORT
		                     DIGIT_Set(dig[4]);
		                     GPIO_Value_Set(COM4_PORT,COM4_PIN,ssd_on);	
		                     #endif	
		                     }break;
		              case (5):/*COM5*/
		                     {
		                     #ifdef COM5_PORT
		                     DIGIT_Set(dig[5]);
		                     GPIO_Value_Set(COM5_PORT,COM5_PIN,ssd_on);	
		                     #endif	
		                     }break;
		              case (6):/*COM6*/
		                     {
		                     #ifdef COM6_PORT
		                     DIGIT_Set(dig[6]);
		                     GPIO_Value_Set(COM6_PORT,COM6_PIN,ssd_on);	
		                     #endif	
		                     }break;
		              case (7):/*COM7*/
		                     {
		                     #ifdef COM7_PORT
		                     DIGIT_Set(dig[7]);
		                     GPIO_Value_Set(COM7_PORT,COM7_PIN,ssd_on);	
		                     #endif	
		                     }break;
                      }
             }
	    
}    
/**END =======================================================================================*/
