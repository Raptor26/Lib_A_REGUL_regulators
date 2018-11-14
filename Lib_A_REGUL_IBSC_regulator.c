/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "Lib_A_REGUL_IBSC_regulator.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
__REGUL_FPT__
REGUL_IBSC(
	regul_ibsc_s *pIBSC_s,
	__REGUL_FPT__ e1,
	__REGUL_FPT__ phi_d,
	__REGUL_FPT__ omega_x)
{
	/* Расчет интегральной составляющей (смотри комментарий к eq. 4.46) */
	pIBSC_s->chi += e1 * pIBSC_s->coeff_s.lambda;

	/* Ограничение насыщения */
	pIBSC_s->chi =
		REGUL_RestrictionSaturation(
			pIBSC_s->chi,
			pIBSC_s->saturation);

	/* Дифференцирование phi_d */
	__REGUL_FPT__ phi_d_deriv =
		DIFF_GetDifferent1(
			&pIBSC_s->phi_d_Deriv_s,
			phi_d);

	/* Расчет желаемой скорости (eq. 4.46) */
	__REGUL_FPT__ omega_xd =
		(pIBSC_s->coeff_s.c1 * e1)
		+ phi_d_deriv
		+ (pIBSC_s->coeff_s.lambda * pIBSC_s->chi);

	omega_xd =
		REGUL_RestrictionSaturation(
			omega_xd,
			pIBSC_s->saturation);

	/* Расчет ошибки между желаемой скоростью и фактической (eq. 4.48) */
	__REGUL_FPT__ e2 =
		omega_xd - omega_x;

	/* Расчет управляющего воздействия (eq. 4.53) */
	__REGUL_FPT__ returnValue =
		((__REGUL_FPT__)1.0 / pIBSC_s->coeff_s.b1)
		* ((((__REGUL_FPT__)1.0 - (pIBSC_s->coeff_s.c1 * pIBSC_s->coeff_s.c1) + pIBSC_s->coeff_s.lambda) * e1)
		   + ((pIBSC_s->coeff_s.c1 + pIBSC_s->coeff_s.c2) * e2)
		   - (pIBSC_s->coeff_s.c1 * pIBSC_s->coeff_s.lambda * pIBSC_s->chi));

	returnValue =
		REGUL_RestrictionSaturation(
			returnValue,
			pIBSC_s->saturation);
	
	return (returnValue);
}

regul_fnc_status_e
REGUL_Init_IBSC(
	regul_ibsc_s		*pIBSC_s,
	regul_ibsc_init_s	*pIBSC_Init_s)
{
	pIBSC_s->initStatus_e = REGUL_ERROR;
	/* Если не задан период дифференцирования */
	if (pIBSC_Init_s->dT == (__REGUL_FPT__) 0.0)
	{
		/* Зависаем в бесконечном цикле */
		while (1);
	}

	/* Инициализация коэффициентов регулятора */
	pIBSC_s->coeff_s.b1			= pIBSC_Init_s->coeff_s.b1;
	pIBSC_s->coeff_s.c1			= pIBSC_Init_s->coeff_s.c1;
	pIBSC_s->coeff_s.c2			= pIBSC_Init_s->coeff_s.c2;
	pIBSC_s->coeff_s.lambda		= pIBSC_Init_s->coeff_s.lambda;

	/* Инициализация остальных параметров регулятора */
	pIBSC_s->chi			= (__REGUL_FPT__) 0.0;
	pIBSC_s->saturation		= pIBSC_Init_s->saturation;

	/* Инициализация структуры для дифференцирования phi_d */
	diff_differentiation_1_init_s diffInit_s;
	DIFF_Different1_StructInit(&diffInit_s);
	diffInit_s.dT = pIBSC_Init_s->dT;
	diff_fnc_status_e diffInitStatus_e =
		DIFF_Init_Different1(
			&pIBSC_s->phi_d_Deriv_s,
			&diffInit_s);
	/* Если функция инициализации вернула ошибку, то "зависаем" в бесконечном цикле */
	while (diffInitStatus_e != DIFF_SUCCESS)
	{
	}

	pIBSC_s->initStatus_e = REGUL_SUCCESS;

	return (pIBSC_s->initStatus_e);
}

void
REGUL_IBSC_StructInit(
	regul_ibsc_init_s *pIBSC_Init_s)
{
	pIBSC_Init_s->coeff_s.b1 		= (__REGUL_FPT__) 0.0;
	pIBSC_Init_s->coeff_s.c1 		= (__REGUL_FPT__) 0.0;
	pIBSC_Init_s->coeff_s.c2 		= (__REGUL_FPT__) 0.0;
	pIBSC_Init_s->coeff_s.lambda 	= (__REGUL_FPT__) 0.0;

	pIBSC_Init_s->dT 				= (__REGUL_FPT__) 0.0;
	pIBSC_Init_s->saturation 		= (__REGUL_FPT__) 0.0;
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
