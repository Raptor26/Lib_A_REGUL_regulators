/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef LIB_A_REGUL_IBSC_REGULATOR_H_
#define LIB_A_REGUL_IBSC_REGULATOR_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
#include <math.h>
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "Lib_A_REGUL_regulators.h"
#include "../Lib_A_DIFF_differentiators/Lib_A_DIFF_differentiators.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/
/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/
typedef struct
{
	/**
	 * @brief Коэффициент коррекции ошибки положения;
	 * @warning Это поле должно быть определено при инициализации структуры;
	 */
	__REGUL_FPT__ c1;

	/**
	 * @brief Коэффициент коррекции ошибки скорости;
	 * @warning   Это поле должно быть определено при инициализации структуры;
	 */
	__REGUL_FPT__ c2;

	/**
	 * @brief     Коэффициент;
	 * @warning   Это поле должно быть определено при инициализации структуры;
	 * @see       eq. 4.53;
	 */
	__REGUL_FPT__ b1;

	/**
	 * @brief Коэффициент коррекции интегральной составляющей ошибки положения;
	 * @warning   Это поле должно быть определено при инициализации структуры;
	 */
	__REGUL_FPT__ lambda;
} regul_ibsc_coeff_s;

typedef struct
{
	regul_ibsc_coeff_s 			coeff_s;

	__REGUL_FPT__ 				saturation;

	/**
	 * @brief Интегральная составляющая от ошибки положения;
	 */
	__REGUL_FPT__ 				chi;

	diff_differentiator_1_s 	phi_d_Deriv_s;

	regul_fnc_status_e 			initStatus_e;
} regul_ibsc_s;

typedef struct
{
	regul_ibsc_coeff_s 	coeff_s;
	__REGUL_FPT__ 		dT;
	__REGUL_FPT__ 		saturation;
} regul_ibsc_init_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
__REGUL_FPT__
REGUL_IBSC(
	regul_ibsc_s *pIBSC_s,
	__REGUL_FPT__ e1,
	__REGUL_FPT__ phi_d,
	__REGUL_FPT__ omega_x);

extern regul_fnc_status_e
REGUL_Init_IBSC(
	regul_ibsc_s		*pIBSC_s,
	regul_ibsc_init_s	*pIBSC_Init_s);

void
REGUL_IBSC_StructInit(
	regul_ibsc_init_s *pIBSC_Init_s);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/

#endif	/* LIB_A_REGUL_IBSC_REGULATOR_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
