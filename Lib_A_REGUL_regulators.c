/*
 * Lib_A_REGUL_regulators.c
 *
 *  Created on: 2 февр. 2018 г.
 *      Author: m.isaev
 */



/******************************************************************************/
// 	Секция include: здесь подключается заголовочный файл к модулю
#include "Lib_A_REGUL_regulators.h"
/******************************************************************************/


/******************************************************************************/
/*============================================================================*/
// 	Глобальные переменные
/*============================================================================*/


/*============================================================================*/
// 	Локальные переменные
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
// 	Секция прототипов локальных функций
/******************************************************************************/


/******************************************************************************/
// 	Секция описания функций (сначала глобальных, потом локальных)
/*============================================================================*/

/**
 * @brief
 * @param	[in]	*pStruct:	Указатель на структуру, содержащую необходимые
 * 								данные для регулятора IBSC;
 * @param	[in]	phi_d:	Желаемое положение;
 * @param	[in]	phi:	Текущее положение;
 * @param	[in]	omega_x:	Производная от регулируемого параметра;
 * @return	Рассчитанная величина управляющего воздействия;
 */
float REGUL_IntegralBackStep(
                             REGUL_integ_back_step_s *pStruct,
                             float phi_d,
                             float phi,
                             float omega_x)
{
	//	Разница между желаемым положением и текущим;
	float e1 = phi_d - phi;

	//	Возведение в степень модуля ошибки e1;
	//	Если коэффициент, в который необходимо возвести "e1" не равен "1.0f" или "0.0f":
	if (pStruct->e1PowCoeff != 1.0f || pStruct->e1PowCoeff != 0.0f)
	{
		//	Если "e1" положительное число;
		if (e1 >= 0.0f)
		{
			e1 = powf(e1, pStruct->e1PowCoeff);
		}
		//	Иначе:
		else
		{
			//	Результат возведения в степень модуля числа "e1" умножается на "-1.0f"
			e1 = (powf(fabsf(e1), pStruct->e1PowCoeff)) * -1.0f;
		}
	}

	//--------------------------------------------------------------------------
	/*	Дифференцирование желаемого положения */
	//	Нахождение производной от желаемого углового положения;
	pStruct->phi_d_deriv = (phi_d - pStruct->phi_d_t1) / pStruct->dT;

	//	Текущее "желаемое положение" копируется в переменную "желаемое положение
	//	на шаге <t-1>" для дифференцирования на при следующем вызове функции;
	pStruct->phi_d_t1 = phi_d;
	//--------------------------------------------------------------------------

	//	Расчет интегральной состоавляющей (смотри комментарий к eq. 4.46);
	pStruct->chi += e1 * pStruct->lambda * pStruct->dT;

	//	Расчет желаемой угловой скорости (eq. 4.46);
	pStruct->omega_xd = pStruct->c1 * e1
	                    + pStruct->phi_d_deriv
	                    + pStruct->lambda * pStruct->chi;

	//	Расчет ошибки между желаемой угловой скоростью и фактической (eq. 4.48);
	float e2 = pStruct->omega_xd - omega_x;

	//	Возведение в степень модуля ошибки e2;
	//	Если коэффициент, в который необходимо возвести "e2" не равен "1.0f" или "0.0f":
	if (pStruct->e2PowCoeff != 1 || pStruct->e2PowCoeff != 0)
	{
		//	Если "e2" положительное число;
		if (e2 >= 0.0f)
		{
			e2 = powf(e2, pStruct->e1PowCoeff);
		}
		//	Иначе:
		else
		{
			//	Результат возведения в степень модуля числа "e2" умножается на "-1.0f"
			e2 = (powf(fabsf(e2), pStruct->e2PowCoeff)) * -1.0f;
		}
	}

	//	Расчет управляющего воздействия (eq. 4.53);
	float returnValue = 1 / pStruct->b1
	                    * (1 - pStruct->c1 * pStruct->c1 + pStruct->lambda)
	                    * e1
	                    + (pStruct->c1 + pStruct->c2) * e2
	                    - pStruct->c1 * pStruct->lambda * pStruct->chi;

	return returnValue;
}

/**
 * @brief	Функция выполняет инициализацию полей структуры "REGUL_integ_back_step_s"
 * @param	[out]	*pStruct:	Указатель на структуру, в поля которой будут записаны
 * 								значения для расчета управляющего воздействия;
 * @param	[in]	dT:	Шаг дифференцирования регулируемого параметра в "сек";
 * @param	[in]	c1:	Коэффициент коррекции ошибки регулируемого параметра;
 * @param	[in]	c2:	Коэффициент коррекции производной от ошибки регулируемого
 * 						параметра;
 * @param	[in]	lambda:	Коэффициент интегральной коррекции;
 * @param	[in]	b1:	Коэффициент коррекции системы;
 * @note	см. eq. 4.45 - 4.53 в документе "Design and control of quadrotors
 *			with application to autonomous flying"
 * @return	None;
 */
void REGUL_Init_IntergralBackStep(
                                  REGUL_integ_back_step_s *pStruct,
                                  float dT,
                                  float c1,
                                  float c2,
                                  float lambda,
                                  float b1)
{
	pStruct->dT = dT;
	pStruct->c1 = c1;
	pStruct->c2 = c2;
	pStruct->lambda = lambda;
	pStruct->b1 = b1;

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
/******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
// 	END OF FILE
////////////////////////////////////////////////////////////////////////////////
