/**
 * @file	Lib_A_REGUL_regulators.c;
 * @author 	Isaev Mickle;
 * @version	beta;
 * @date 	15.02.2018;
 * @brief	Библиотека содержит следующие функции регуляторов:
 * 			+ Инициализация структуры REGUL_integ_back_step_s стандартными значениями;
 * 			+ Расчет управляющего воздействия методом IntegralBackStep;
 * 			+ Расчет управляющего воздействия методом PI;
 */



/******************************************************************************/
// 	Секция include: здесь подключается заголовочный файл к модулю
#include "Lib_A_REGUL_regulators.h"
/******************************************************************************/


/******************************************************************************/
/*============================================================================*/
// 	Глобальные переменные
#if	defined __REGUL_REGULATORS_DEBUG__
float g_e1;
float g_e2;
float g_IntegralBackStepReturnValue;
float g_omega_x;
float g_chi;
float g_b1;
#endif
/*============================================================================*/


/*============================================================================*/
// 	Локальные переменные
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
// 	Секция прототипов локальных функций
float PowerForIBSC(
                   float basis,
                   float exponent);
float RestrictionSaturation(
                            float value,
                            float saturation);
/******************************************************************************/


/******************************************************************************/
// 	Секция описания функций (сначала глобальных, потом локальных)
/*============================================================================*/

/**
 * @brief	Функция для расчета управляющего воздействия с помощью
 * 			"Integral Back Step Controller";
 * @param[in]	*pStruct:	Указатель на структуру, содержащую необходимые
 * 							данные для регулятора IBSC;
 * @param[in]	phi_d:		Желаемое положение;
 * 		@note	Если "phi_d_its_e1Flag" == 1, то "phi_d" интерпретируется как ошибка
 * 				между желаемым положнием и текущим (т.е. как "e1");
 * 		@see	REGUL_integ_back_step_s;
 *
 * @param[in]	phi:		Текущее положение;
 * 		@note	Если "phi_d_its_e1Flag" == 1, то переменная "phi" не используется в
 * 				функции "REGUL_IntegralBackStep";
 * 		@note	Если "phi_d_its_e1Flag" == 1, то
 * @param[in]	omega_x:	Производная от "phi";
 *
 * @return	Рассчитанная величина управляющего воздействия;
 *
 * @note	см. eq. 4.45 - 4.53 в документе "Design and control of quadrotors
 *			with application to autonomous flying"
 */
float REGUL_IntegralBackStep(
    REGUL_integ_back_step_s *pStruct,
    float e1,
    float phi_d,
    float omega_x)
{
	/*---- |Begin| --> Дифференцирование phi_d (т.е. желаемого положения) ----*/
	/* Установка периода дифференцирования */
	pStruct->phi_d_derivStruct.dT = pStruct->dT;

	/* Дифференцирование желаемого положения методом 1-го порядка */
	float phi_d_deriv = DIFF_FindDifferent1(*pStruct->phi_d_derivStruct,
	                                        phi_d);
	/*---- |End  | <-- Дифференцирование phi_d (т.е. желаемого положения) ----*/

	/* Расчет интегральной состоавляющей (смотри комментарий к eq. 4.46) */
	pStruct->chi += e1 * pStruct->lambda * pStruct->dT;

	/*	Ограничение насыщения интегральной составляющей */
	pStruct->chi = RestrictionSaturation(pStruct->chi,
	                                     pStruct->saturation);

	/* Расчет желаемой скорости (eq. 4.46) */
	float omega_xd = pStruct->c1 * e1
	                    + phi_d_deriv
	                    + pStruct->lambda * pStruct->chi;

	/* Ограничение насыщения желаемой скорости */
	omega_xd = RestrictionSaturation(omega_xd,
	                                 pStruct->saturation);
	
	/* Расчет ошибки между желаемой скоростью и фактической (eq. 4.48) */
	float e2 = omega_xd - omega_x;

	/* Коэффициент "b1" должен быть только положительным */
	if (pStruct->b1 < 0.0f)
	{
		pStruct->b1 *= -1.0f;
	}

	/* Расчет управляющего воздействия (eq. 4.53) */
	float returnValue = (1 / pStruct->b1)
	                    * (1 - pStruct->c1 * pStruct->c1 + pStruct->lambda)
	                    * e1
	                    + ((pStruct->c1 + pStruct->c2) * e2)
	                    - (pStruct->c1 * pStruct->lambda * pStruct->chi);

	/* Ограничение насыщения выходного параметра */
	returnValue = RestrictionSaturation(returnValue,
	                                    pStruct->saturation);

#if	defined __REGUL_REGULATORS_DEBUG__
	g_e1 = e1;
	g_e2 = e2;
	g_IntegralBackStepReturnValue = returnValue;
	g_omega_x = omega_x;
	g_chi = pStruct->chi;
	g_b1 = pStruct->b1;
#endif

	return returnValue;
}

/**
 * @brief	Функция выполняет инициализацию полей структуры "REGUL_integ_back_step_s"
 * @param[out]	*pStruct:	Указатель на структуру, в поля которой будут записаны
 * 							стандартные параметры регулятора;
 *
 * @return	None;
 *
 * @note	см. eq. 4.45 - 4.53 в документе "Design and control of quadrotors
 *			with application to autonomous flying"
 */
void REGUL_Init_IntergralBackStep(
                                  REGUL_integ_back_step_s *pStruct)
{
	pStruct->dT = 0.0f;
	pStruct->c1 = 0.0f;
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

	/* Значения переключателей */
	pStruct->tumblers.e1TakeModuleFlag = REGUL_DIS;
	pStruct->tumblers.phi_d_its_e1Flag = REGUL_DIS;
	pStruct->tumblers.enablePowFunctFlag = REGUL_DIS;
}

/**
 * @brief	Функция ПИ регулятора с ограничением насыщения интегральной составляющей;
 * @see	А.С. Анучин "Системы управления электроприводами" стр. 209
 */
float REGUL_PI_regulator(REGUL_pid_regulator_s *pSturct, float error)
{
	float propCorrect;
	float piCorrect;
	float errorAndSaturetionDiff;

	//	Расчет пропорциональной коррекции
	propCorrect = error * pSturct->kp;

	//	Расчет интегральной коррекции;
	pSturct->integral += error * pSturct->ki * pSturct->dT;

	//	Расчет ПИ регулятора;
	piCorrect = propCorrect + pSturct->integral;

	piCorrect += pSturct->integralAfterCorrect;

	// Рассчет во сколько раз сигнал получился больше насыщения;
	errorAndSaturetionDiff = fabsf(piCorrect) / fabsf(pSturct->saturation);

	//	Масштабирование входного сигнала;
//	error *= errorAndSaturetionDiff;

	//	Расчет пропорциональной коррекции после коррекции насыщения
	propCorrect = error * pSturct->kp;

	//	Расчет интегральной коррекции;
	pSturct->integralAfterCorrect += error * pSturct->ki * pSturct->dT;

	//	Ограничение насыщения интегральной составляющей;
	if (pSturct->integralAfterCorrect > pSturct->saturation)
	{
		pSturct->integralAfterCorrect = pSturct->saturation;
	}
	else if (pSturct->integralAfterCorrect < (-pSturct->saturation))
	{
		pSturct->integralAfterCorrect = -pSturct->saturation;
	}

	//	Расчет ПИ регулятора;
	piCorrect = propCorrect + pSturct->integral;

	//	Ограничение насыщения ПИ регулятора;
	if (piCorrect > pSturct->saturation)
	{
		piCorrect = pSturct->saturation;
	}
	else if (piCorrect < (-pSturct->saturation))
	{
		piCorrect = -pSturct->saturation;
	}

	return piCorrect;
}
/*============================================================================*/

/* Локальные функции */

/**
 * @brief
 * @param
 * @rapam
 * @return
 */
float PowerForIBSC(
                   float basis,
                   float exponent)
{
	//	Если степень, в которую необходимо возвести "basis" не равна "1.0f",
	//	"-1.0f" или "0.0f":
	if ((exponent != 1.0f)
	    || (exponent != -1.0f)
	    || (exponent != 0.0f))
	{
		//	Если "basis" положительное число;
		if (basis >= 0.0f)
		{
			//	"e1PowCoeff" берется по модулю;
			basis = powf(basis, fabsf(exponent));
		}
		//	Иначе (если "basis" отрицательное число):
		else
		{
			//	Результат возведения в степень модуля числа "basis" умножается на "-1.0f";
			//	"exponent" берется по модулю;
			basis = (powf(fabsf(basis), fabsf(exponent))) * -1.0f;
		}
	}
	return basis;
}

/**
 * @brief	Функция проверяет ограничение насыщения входного параметра по
 * 			положительному и отрицательному пределу насыщения;
 * @param[in]	value:	Переменная, насыщение которой необходимо проверить;
 * @param[in]	saturation:	Значение насыщения, с которым необходимо сравнить
 * 							переменную "value"
 * @return	Значение переменной "value", с учетом значения насыщения "saturation";
 */
float RestrictionSaturation(
                            float value,
                            float saturation)
{
	/*	Ограничение насыщения выходного параметра */
	//	Если переменная насыщения не равна "0.0f":
	if (saturation != 0.0f)
	{
		//	Если выходное значение больше положительного значения переменной насыщения:
		if (value > saturation)
		{
			value = saturation;
		}
		//	Если выходное значение меньше отрицательного значения переменной насыщения:
		else if (value < (-saturation))
		{
			value = -saturation;
		}
	}
	return value;
}
/******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
// 	END OF FILE
////////////////////////////////////////////////////////////////////////////////
