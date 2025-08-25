/********************************************************************************************************************************************/
/*  Author      : Ahmed Magdy                                                                                                            */
/*  Module      : Component_Buzzer_H                                                                                                                       */
/*  Last Update : 7 / 11 / 2023                                                                                                            */
/********************************************************************************************************************************************/
#ifndef Component_Buzzer_H_
#define Component_Buzzer_H_
/**Includes ==================================================================================*/

#include "MCAL_BUZZER_OUTPUT.h"
/**Definitions ===============================================================================*/
#define BUZ_Time (25)
/**Selections ================================================================================*/
typedef enum {
	buz,
	alarm,
	end,
	mute
}t_tone;
/**Prototypes ================================================================================*/
/* Buzzer_Init( BUZZ0 or BUZZ1  );  //buzz0 P31  ,buzz1 P15   */
  void Buzzer_Init(tbyte  NUMBER_BUZZER  );

  /* @brief :*/
  void Buze_Tone_start();
  /* @brief :*/
  void Buze_Tone_Stop();
 /* @brief :*/
  void Buze_Tone_Set(t_tone tone ) ;

/* @brief :*//* @brief Buzzer_Update(0,200);:*/
  void Buzzer_Update(void) ;

/**END =======================================================================================*/
#endif
