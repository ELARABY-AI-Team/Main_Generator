/********************************************************************************************************************************************/
/*  Author      : Ahmed Magdy                                                                                                              */
/*  Module      : Component_Buzzer_C                                                                                                                      */
/*  Last Update : 01 / 01 / 2023                                                                                                            */
/********************************************************************************************************************************************/
/**Includes ==================================================================================*/
#include "MCAL_General_Config.h"
#include "HAL_BUZZER.h"
#include "HAL_BUZZER_USER.h"
/**Static Variables ==================================================================================*/
static tbyte buz_i;
static tbyte buz_n;
static tbyte buz_r;
static tbyte Buzzer = 0 ;
//static tbyte buz_tick_i;
/**Functions ==================================================================================*/
void Buzzer_Variables_Reset()
  {
  	buz_i=0;
  	buz_r=0;
  }
/* @brief :*/
void Buzzer_Init(tbyte  NUMBER_BUZZER)
  {
  	WDT_Reset();
  	Buzzer_Variables_Reset();
  	buz_n=0;
   Buzzer = NUMBER_BUZZER ;
    BUZZER_OUTPUT_Init(NUMBER_BUZZER);
  	
  	
  }
  /* @brief :*/
  void Buze_Tone_start()
  {
	  	WDT_Reset();
  	BUZZER_OUTPUT_Enable(Buzzer) ;
  }
  /* @brief :*/
  void Buze_Tone_Stop()
  {
	  	WDT_Reset();
  	BUZZER_OUTPUT_Disable(Buzzer) ;
  	   }
 /* @brief :*/
  void Buze_Tone_Set(t_tone tone )
  {
  		WDT_Reset();
  	if(tone==mute)
  	             {buz_n=0;buz_i=0;Buze_Tone_Stop();}
  	 else
  	     {
  	     if(tone==buz)
  	             {buz_n=1;buz_i=0;Buze_Tone_start();}
  	     else
  	         {
  	         if(tone==end)
  	             {if((buz_n==3)||(buz_n==33)){/*Do Nothing*/}
  	              else{buz_n=3;buz_i=0;buz_r=0;Buze_Tone_start();}}
  	             else
  	                 {
  	                 if(tone==alarm)
  	                    {
  	                    if((buz_n==2)||(buz_n==22)){/*Do Nothing*/}
  	                    else{buz_n=2;buz_i=0;Buze_Tone_start();}
  	                    }
  	                 }                            
  	         }
  	     }            
  }
/* @brief :*/
  void Buzzer_Update(void)
  {
  		WDT_Reset();

  	 switch(buz_n)
  	   {
        case (0):/*MUTE*/
             {
             Buzzer_Variables_Reset();
             Buze_Tone_Stop();	
             }break;
       case (1):/*BUZ*/
             {
             buz_i++;
             switch(buz_i)
                 {
                 	case(1):{buz_n=0;}break;
                 }	
             }break;
       case (2):/*ALARM_BUZ*/
             {
             buz_i++;
             switch(buz_i)
                 {
                 	case(1):{buz_i=0;buz_n=22;Buze_Tone_Stop();}break;
                 }	
             }break;
       case (22):/*ALARM_MUTE*/
             {
             buz_i++;
             switch(buz_i)
                 {
                 	case(1):{buz_i=0;buz_n=2;Buze_Tone_start();}break;
                 }	
             }break;
       case (3):/*END_BUZ*/
             {
             buz_i++;
             switch(buz_i)
                 {
                 	case(5):{buz_r++;
                                 	buz_i=0;
                                 	Buze_Tone_Stop();
                 	                if(buz_r>=3){buz_n=0;}else{buz_n=33;}}break;
                 }	
             }break;
       case (33):/*END_MUTE*/
             {
             buz_i++;
             switch(buz_i)
                 {
                 	case(10):{buz_i=0;buz_n=3;Buze_Tone_start();}break;
                 }	
             }break;
  	}

  }

/**END ==================================================================================*/
