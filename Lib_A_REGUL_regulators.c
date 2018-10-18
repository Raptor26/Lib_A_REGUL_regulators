/**
 * @file    Lib_A_REGUL_regulators.c;
 * @author  Isaev Mickle;
 * @version beta;
 * @date    15.02.2018;
 * @brief   Библиотека содержит следующие функции регуляторов:
 *          + Инициализация структуры REGUL_integ_back_step_s стандартными значениями;
 *          + Расчет управляющего воздействия методом IntegralBackStep;
 *          + Расчет управляющего воздействия методом PI;
 */



/******************************************************************************/
//  Секция include: здесь подключается заголовочный файл к модулю
#include "Lib_A_REGUL_regulators.h"
/******************************************************************************/


/******************************************************************************/
/*============================================================================*/
//  Глобальные переменные
#if defined __REGUL_REGULATORS_DEBUG__
__REGUL_FPT__ g_e1;
__REGUL_FPT__ g_e2;
__REGUL_FPT__ g_IntegralBackStepReturnValue;
__REGUL_FPT__ g_omega_x;
__REGUL_FPT__ g_omega_xd;
__REGUL_FPT__ g_phi_d_deriv;
__REGUL_FPT__ g_chi;
__REGUL_FPT__ g_b1;
#endif
/*============================================================================*/


/*============================================================================*/
//  Локальные переменные
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
//  Секция прототипов локальных функций
__REGUL_FPT__
RestrictionSaturation (
	__REGUL_FPT__ value,
	__REGUL_FPT__ saturation);
/******************************************************************************/


/******************************************************************************/
//  Секция описания функций (сначала глобальных, потом локальных)
/*============================================================================*/

/**
 * @brief   Функция для расчета управляющего воздействия с помощью
 *          "Integral Back Step Controller";
 * @param[in]   *pStruct:   Указатель на структуру, содержащую необходимые
 *                          данные для регулятора IBSC;
 * @param[in]   phi_d:      Желаемое положение;
 *      @note   Если "phi_d_its_e1Flag" == 1, то "phi_d" интерпретируется как ошибка
 *              между желаемым положнием и текущим (т.е. как "e1");
 *      @see    REGUL_integ_back_step_s;
 *
 * @param[in]   phi:        Текущее положение;
 *      @note   Если "phi_d_its_e1Flag" == 1, то переменная "phi" не используется в
 *              функции "REGUL_IntegralBackStep";
 *      @note   Если "phi_d_its_e1Flag" == 1, то
 * @param[in]   omega_x:    Производная от "phi";
 *
 * @return  Рассчитанная величина управляющего воздействия;
 *
 * @note    см. eq. 4.45 - 4.53 в документе "Design and control of quadrotors
 *          with application to autonomous flying"
 */
__REGUL_FPT__
REGUL_Get_IBSC (
	regul_ibsc_s *pStruct,
	__REGUL_FPT__ e1,
	__REGUL_FPT__ phi_d,
	__REGUL_FPT__ omega_x)
{
	/*---- |Begin| --> Дифференцирование phi_d (т.е. желаемого положения) ----*/
	/* Установка периода дифференцирования */
	pStruct->phi_d_derivStruct.dT = pStruct->dT;

	/* Дифференцирование желаемого положения методом 1-го порядка */
	__REGUL_FPT__ phi_d_deriv =
		DIFF_GetDifferent1 (
			&pStruct->phi_d_derivStruct,
			phi_d);
	/*---- |End  | <-- Дифференцирование phi_d (т.е. желаемого положения) ----*/

	/* Расчет интегральной состоавляющей (смотри комментарий к eq. 4.46) */
	pStruct->chi +=
		e1 * pStruct->lambda * pStruct->dT;

	/*    Ограничение насыщения интегральной составляющей */
	pStruct->chi =
		RestrictionSaturation (
			pStruct->chi,
			pStruct->saturation);

	/* Расчет желаемой скорости (eq. 4.46) */
	__REGUL_FPT__ omega_xd =
		pStruct->c1 * e1
		+ phi_d_deriv
		+ pStruct->lambda * pStruct->chi;

	/* Ограничение насыщения желаемой скорости */
	omega_xd =
		RestrictionSaturation (
			omega_xd,
			pStruct->saturation);

	/* Расчет ошибки между желаемой скоростью и фактической (eq. 4.48) */
	__REGUL_FPT__ e2 = omega_xd - omega_x;

	/* Коэффициент "b1" должен быть только положительным */
	if (pStruct->b1 < 0.0f)
	{
		pStruct->b1 *= -1.0f;
	}

	/* Расчет управляющего воздействия (eq. 4.53) */
	__REGUL_FPT__ returnValue =
		(1 / pStruct->b1)
		* (1 - pStruct->c1 * pStruct->c1 + pStruct->lambda)
		* e1
		+ ((pStruct->c1 + pStruct->c2) * e2)
		- (pStruct->c1 * pStruct->lambda * pStruct->chi);

	/* |Begin| --> Взятие второй производной от e1 ---------------------------*/
	pStruct->e1_FirstDerivStruct.dT = pStruct->dT;
	pStruct->e1_SecontDerivStruct.dT = pStruct->dT;

	/* Нахождение первой производной от e1 */
	__REGUL_FPT__ e1DerivTemp =
		DIFF_GetDifferent1 (
			&pStruct->e1_FirstDerivStruct,
			e1);

	/* Нахождение второй производной от e1 */
	//    e1DerivTemp = DIFF_FindDifferent1(&pStruct->e1_SecontDerivStruct,
	//                                      e1DerivTemp);

	/* Добавление второй производной от ошибки положения в результат
	 * работы Integral Back Step Control */
	returnValue += e1DerivTemp * pStruct->e1SecondDerivCoeff;
	/* |End  | <-- Взятие второй производной от e1 ---------------------------*/

	/* Ограничение насыщения выходного параметра */
	returnValue =
		RestrictionSaturation (
			returnValue,
			pStruct->saturation);

#if defined __REGUL_REGULATORS_DEBUG__
	g_e1 = e1;
	g_e2 = e2;
	g_IntegralBackStepReturnValue = returnValue;
	g_omega_x = omega_x;
	g_omega_xd = omega_xd;
	g_phi_d_deriv = phi_d_deriv;
	g_chi = pStruct->chi;
	g_b1 = pStruct->b1;
#endif

	return returnValue;
}

/**
 * @brief   Функция выполняет инициализацию полей структуры "REGUL_integ_back_step_s"
 * @param[out]  *pStruct:   Указатель на структуру, в поля которой будут записаны
 *                          стандартные параметры регулятора;
 *
 * @return  None;
 *
 * @note    см. eq. 4.45 - 4.53 в документе "Design and control of quadrotors
 *          with application to autonomous flying"
 */
void
REGUL_Init_IBSC (
	regul_ibsc_s *pStruct)
{
	pStruct->dT = 0.0f;
	pStruct->c1 = 1.0f;
	pStruct->c2 = 0.0f;
	pStruct->lambda = 0.0f;
	pStruct->b1 = 1.0f;
	pStruct->e1PowCoeff = 1.0f;
	pStruct->e2PowCoeff = 1.0f;
	pStruct->saturation = 0.0f;
	pStruct->chi = 0.0f;
	pStruct->omega_xd = 0.0f;
	pStruct->phi_d_t1 = 0.0f;
	pStruct->phi_d_deriv = 0.0f;
	pStruct->e1SecondDerivCoeff = 0.0f;

	/* Значения переключателей */
	pStruct->tumblers.e1TakeModuleFlag = REGUL_DIS;
	pStruct->tumblers.phi_d_its_e1Flag = REGUL_DIS;
	pStruct->tumblers.enablePowFunctFlag = REGUL_DIS;
}

/**
 * @brief	Функция выполняет расчет управляющего воздействия с помощью 
 *        	ПИД регулятора
 * @param[in,out]	*pPID_s:	Указатель на структуру ПИД регулятора
 * @param[in]	err:	Ошибка, по которой производится расчет управляющего 
 * 						воздействия
 * @param[in]	errDeriv:	Дифференциальная составляющая ошибки.
 * 							Если errDeriv = NULL, то функция ПИД регулятора 
 * 							самостоятельно выполняет дифференцирование 
 * 							переменной err методом 1-го порядка
 * @return 	Величина управляющего воздействия
 */
__REGUL_FPT__
REGUL_Get_PID(
	regul_pid_s *pPID_s,
	__REGUL_FPT__ err,
	__REGUL_FPT__ errDeriv)
{
	/* Применение пропорциональной коррекции */
	__REGUL_FPT__ returnVal =
		err * pPID_s->proportional_s.kP;

	/* Обновление интегральной коррекции (методом трапеций) */
	pPID_s->integral_s.val +=
		(__REGUL_FPT__) NINTEG_Trapz(
			&pPID_s->integral_s.deltaTrap_s,
			err * pPID_s->integral_s.kI);

	/* Ограничение насыщения интегральной коррекции */
	pPID_s->integral_s.val =
		RestrictionSaturation(
			pPID_s->integral_s.val,
			pPID_s->integral_s.satur);

	/* Применение интегральной коррекции */
	returnVal += pPID_s->integral_s.val;

	/* Если значение дифференциальной составляющей регулятора равно нулю */
	if (errDeriv == NULL)
	{
		/* Расчет дифференциальной составляющей регулятора и её применение */
		returnVal +=
			(__REGUL_FPT__) DIFF_GetDifferent1(
				&pPID_s->derivative_s.different1_s,
				err);
	}
	/* Иначе, если значение дифференциальной составляющей передано в функцию */
	else
	{
		/* Применение дифференциальной коррекции */
		returnVal += errDeriv * pPID_s->derivative_s.kD;
	}

	/* Ограничение насыщения возвращаемого регулятором значения */
	returnVal =
		RestrictionSaturation (
			returnVal,
			pPID_s->pidValSatur);

	return returnVal;
}

//void REGUL_Init_PID(
//	regul_pid_s *pDID_s,
//	__REGUL_FLOAT_POINT_TYPE__ kP,
//	__REGUL_FLOAT_POINT_TYPE__ kI,
//	__REGUL_FLOAT_POINT_TYPE__ kD,
//	__REGUL_FLOAT_POINT_TYPE__ dT,
//	__REGUL_FLOAT_POINT_TYPE__ valSatur,
//	__REGUL_FLOAT_POINT_TYPE__ integValSaturation)
//{
//	pDID_s->proportional_s.kP = kP;
//	pDID_s->integral_s.kI = kI;
//	pDID_s->derivative_s.kD = kD;
//	pDID_s->dT = dT;
//	pDID_s->integral_s.satur = integValSaturation;
//	pDID_s->pidValSatur = valSatur;
//	pDID_s->integral_s.val = (__REGUL_FLOAT_POINT_TYPE__) 0.0;
//	pDID_s->integral_s.deltaTrap_s.dT = dT;
//	pDID_s->integral_s.deltaTrap_s.tumblers_s.accumEn = 1;
//}

/**
 * @brief	Функция выполняет инициализацию структуры ПИД регулятора
 * @param[out]	*p_s:	Указатель на структуру ПИД регулятора
 * @param[in]	*pInit_s:	Указатель на структуру, в которой содержатся
 *							параметры для инициализации структуры ПИД регулятора
 * @return	regul_fnc_status_e:
 *			- REGUL_ERROR:	Если неверно задан один из параметров структуры
 *							regul_pid_init_struct_s
 *			- REGUL_SUCCESS:		Инициализация прошла успешно
 */
regul_fnc_status_e
REGUL_Init_PID(
	regul_pid_s *p_s,
	regul_pid_init_struct_s *pInit_s)
{
	/* Инициализация структуры ПИД регулятора*/
	regul_fnc_status_e pidInitStatus_e;
	if ((pInit_s->dT != (__REGUL_FPT__) 0.0)
			&& (pInit_s->returnValSaturation != (__REGUL_FPT__) 0.0))
	{
		p_s->dT					= pInit_s->dT;
		p_s->derivative_s.kD	= pInit_s->kD;
		p_s->integral_s.kI		= pInit_s->kI;
		p_s->integral_s.satur	= pInit_s->integralValSaturation;
		p_s->pidValSatur		= pInit_s->returnValSaturation;
		p_s->proportional_s.kP	= pInit_s->kP;
		pidInitStatus_e =
			REGUL_SUCCESS;
	}
	else
	{
		return (pidInitStatus_e = REGUL_ERROR);
	}

	/* Инициализация структуры для интегральной составляющей PID регулятора */
	ninteg_trapz_InitStruct_s trapzInit_s;
	NINTEG_Trapz_StructInit(
		&trapzInit_s);

	/* Инициализация параметров структуры */
	trapzInit_s.accumulate_flag = 1u;
	trapzInit_s.integratePeriod = pInit_s->dT;

	/* Инициализация структуры интегрирования */
	ninteg_fnc_status_e trapzInitStatus_e =
		NINTEG_Trapz_Init(
			&p_s->integral_s.deltaTrap_s,
			&trapzInit_s);
	pidInitStatus_e = trapzInitStatus_e;

	/* Инициализация структуры для дифференцирования */
	diff_differentiation_1_init_struct_s diffInit_s;
	DIFF_Different1_StructInit(
		&diffInit_s);

	/* Инициализация параметров структуры */
	diffInit_s.dT = pInit_s->dT;

	/* Инициализация структуры дифференцирования */
	diff_fnc_status_e different1InitStatus_e = DIFF_ERROR;
	different1InitStatus_e =
		DIFF_Init_Different1(
			&p_s->derivative_s.different1_s,
			&diffInit_s);
	pidInitStatus_e = different1InitStatus_e;

	/* Возврат статуса инициализации */
	return (pidInitStatus_e);
}

void
REGUL_PID_StructInit(
	regul_pid_init_struct_s *pInitStruct)
{
	pInitStruct->dT =						(__REGUL_FPT__) 0.0;
	pInitStruct->integralValSaturation =	(__REGUL_FPT__) 0.0;
	pInitStruct->kD =						(__REGUL_FPT__) 0.0;
	pInitStruct->kI =						(__REGUL_FPT__) 0.0;
	pInitStruct->kP =						(__REGUL_FPT__) 0.0;
	pInitStruct->returnValSaturation =		(__REGUL_FPT__) 0.0;
}

__REGUL_FPT__
REGUL_MixTwoVal(
	__REGUL_FPT__ firstVal,
	__REGUL_FPT__ secondVal,
	__REGUL_FPT__ mixCoeff)
{
	if (mixCoeff > (__REGUL_FPT__) 1.0)
	{
		mixCoeff = (__REGUL_FPT__) 1.0;
	}
	if (mixCoeff < (__REGUL_FPT__) 0.0)
	{
		mixCoeff = (__REGUL_FPT__) 1.0;
	}

	return (firstVal * mixCoeff)
		   + (((__REGUL_FPT__) 1.0 - mixCoeff) * secondVal);
}
/*============================================================================*/

/* Локальные функции */

/**
 * @brief
 * @param
 * @rapam
 * @return
 */
__REGUL_FPT__
REGUL_PowerFunc (
	__REGUL_FPT__ basis,
	__REGUL_FPT__ exponent)
{
	//    Если степень, в которую необходимо возвести "basis" не равна "1.0f",
	//    "-1.0f" или "0.0f":
	if ((exponent != 1.0f)
			|| (exponent != -1.0f)
			|| (exponent != 0.0f))
	{
		//    Если "basis" положительное число;
		if (basis >= 0.0f)
		{
			//    "e1PowCoeff" берется по модулю;
			basis = powf (basis, fabsf (exponent));
		}
		//  Иначе (если "basis" отрицательное число):
		else
		{
			//    Результат возведения в степень модуля числа "basis" умножается на "-1.0f";
			//    "exponent" берется по модулю;
			basis = (powf (fabsf (basis), fabsf (exponent))) * -1.0f;
		}
	}
	return basis;
}

/**
 * @brief   Функция проверяет ограничение насыщения входного параметра по
 *          положительному и отрицательному пределу насыщения;
 * @param[in]   value:  Переменная, насыщение которой необходимо проверить;
 * @param[in]   saturation: Значение насыщения, с которым необходимо сравнить
 *                          переменную "value"
 * @return  Значение переменной "value", с учетом значения насыщения "saturation";
 */
__REGUL_FPT__
RestrictionSaturation (
	__REGUL_FPT__ value,
	__REGUL_FPT__ saturation)
{
	/*    Ограничение насыщения выходного параметра */
	//    Если переменная насыщения не равна "0.0":
	if (saturation != (__REGUL_FPT__) 0.0f)
	{
		//    Если выходное значение больше положительного значения переменной насыщения:
		if (value > saturation)
		{
			value = saturation;
		}
		//  Если выходное значение меньше отрицательного значения переменной насыщения:
		else if (value < (-saturation))
		{
			value = -saturation;
		}
	}
	return value;
}
/******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
//  END OF FILE
////////////////////////////////////////////////////////////////////////////////
