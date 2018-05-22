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
float g_e1;
float g_e2;
float g_IntegralBackStepReturnValue;
float g_omega_x;
float g_omega_xd;
float g_phi_d_deriv;
float g_chi;
float g_b1;
#endif
/*============================================================================*/


/*============================================================================*/
//  Локальные переменные
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
//  Секция прототипов локальных функций
REGUL_FLOAT_POINT_TYPE
RestrictionSaturation (
	REGUL_FLOAT_POINT_TYPE value,
	REGUL_FLOAT_POINT_TYPE saturation);
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
float
REGUL_Get_IBSC (
	regul_ibsc_s *pStruct,
	float e1,
	float phi_d,
	float omega_x)
{
	/*---- |Begin| --> Дифференцирование phi_d (т.е. желаемого положения) ----*/
	/* Установка периода дифференцирования */
	pStruct->phi_d_derivStruct.dT = pStruct->dT;

	/* Дифференцирование желаемого положения методом 1-го порядка */
	float phi_d_deriv =
		DIFF_FindDifferent1 (
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
	float omega_xd =
		pStruct->c1 * e1
		+ phi_d_deriv
		+ pStruct->lambda * pStruct->chi;

	/* Ограничение насыщения желаемой скорости */
	omega_xd =
		RestrictionSaturation (
			omega_xd,
			pStruct->saturation);

	/* Расчет ошибки между желаемой скоростью и фактической (eq. 4.48) */
	float e2 = omega_xd - omega_x;

	/* Коэффициент "b1" должен быть только положительным */
	if (pStruct->b1 < 0.0f)
	{
		pStruct->b1 *= -1.0f;
	}

	/* Расчет управляющего воздействия (eq. 4.53) */
	float returnValue =
		(1 / pStruct->b1)
		* (1 - pStruct->c1 * pStruct->c1 + pStruct->lambda)
		* e1
		+ ((pStruct->c1 + pStruct->c2) * e2)
		- (pStruct->c1 * pStruct->lambda * pStruct->chi);

	/* |Begin| --> Взятие второй производной от e1 ---------------------------*/
	pStruct->e1_FirstDerivStruct.dT = pStruct->dT;
	pStruct->e1_SecontDerivStruct.dT = pStruct->dT;

	/* Нахождение первой производной от e1 */
	float e1DerivTemp =
		DIFF_FindDifferent1 (
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

REGUL_FLOAT_POINT_TYPE
REGUL_Get_PID(
	regul_pid_s *pPID_s,
	REGUL_FLOAT_POINT_TYPE err,
	REGUL_FLOAT_POINT_TYPE errDeriv)
{
	/* Применение пропорциональной коррекции */
	REGUL_FLOAT_POINT_TYPE returnVal = err * pPID_s->kP;

	/* Обновление интегральной коррекции */
	pPID_s->integVal += err * pPID_s->kI;

	/* Ограничение насыщения интегральной коррекции */
	pPID_s->integVal =
		RestrictionSaturation(
			pPID_s->integVal,
			pPID_s->integValSaturation);

	/* Применение интегральной коррекции */
	returnVal += pPID_s->integVal;

	/* Применение дифференциальной коррекции */
	returnVal += errDeriv * pPID_s->kD;

	/* Ограничение насыщения возвращаемого регулятором значения */
	returnVal =
		RestrictionSaturation (
			returnVal,
			pPID_s->returnValSaturation);

	return returnVal;
}

void REGUL_Init_PID(
	regul_pid_s *pDID_s,
	REGUL_FLOAT_POINT_TYPE kP,
	REGUL_FLOAT_POINT_TYPE kI,
	REGUL_FLOAT_POINT_TYPE kD,
	REGUL_FLOAT_POINT_TYPE dT,
	REGUL_FLOAT_POINT_TYPE returnValSaturation,
	REGUL_FLOAT_POINT_TYPE integValSaturation)
{
	pDID_s->kP = kP;
	pDID_s->kI = kI;
	pDID_s->kD = kD;
	pDID_s->dT = dT;
	pDID_s->integValSaturation = integValSaturation;
	pDID_s->returnValSaturation = returnValSaturation;
	pDID_s->integVal = (REGUL_FLOAT_POINT_TYPE) 0.0;
}

REGUL_FLOAT_POINT_TYPE
REGUL_MixTwoVal(
	REGUL_FLOAT_POINT_TYPE firstVal,
	REGUL_FLOAT_POINT_TYPE secondVal,
	REGUL_FLOAT_POINT_TYPE mixCoeff)
{
	if (mixCoeff > (REGUL_FLOAT_POINT_TYPE) 1.0)
	{
		mixCoeff = (REGUL_FLOAT_POINT_TYPE) 1.0;
	}
	if (mixCoeff < (REGUL_FLOAT_POINT_TYPE) 0.0)
	{
		mixCoeff = (REGUL_FLOAT_POINT_TYPE) 1.0;
	}

	return (firstVal * mixCoeff)
		   + (((REGUL_FLOAT_POINT_TYPE) 1.0 - mixCoeff) * secondVal);
}
/*============================================================================*/

/* Локальные функции */

/**
 * @brief
 * @param
 * @rapam
 * @return
 */
float
REGUL_PowerFunc (
	float basis,
	float exponent)
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
REGUL_FLOAT_POINT_TYPE
RestrictionSaturation (
	REGUL_FLOAT_POINT_TYPE value,
	REGUL_FLOAT_POINT_TYPE saturation)
{
	/*    Ограничение насыщения выходного параметра */
	//    Если переменная насыщения не равна "0.0":
	if (saturation != (REGUL_FLOAT_POINT_TYPE) 0.0)
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
