/**
 * Lib_A_REGUL_regulators.h
 *
 *  Created on: 2 февр. 2018 г.
 *      Author: m.isaev
 */

#ifndef LIB_A_REGUL_REGULATORS_H_
#define LIB_A_REGUL_REGULATORS_H_


/******************************************************************************/
//  Секция include (подключаем заголовочные файлы используемых модулей)
/*============================================================================*/
//  Стандартные библиотеки языка С
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/*============================================================================*/


/*============================================================================*/
//  Библиотеки для работы с периферией микроконтроллера
/*============================================================================*/


/*============================================================================*/
//  Внешние модули
#include 	"../Lib_A_DIFF_differentiators/Lib_A_DIFF_differentiators.h"
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
//  Секция определения констант
#define	REGUL_EN									1
#define REGUL_DIS									0
/******************************************************************************/


/******************************************************************************/
//  Секция определения типов
/**
 * @brief	Структура, содержащая переменные для работы IBSC регулятора;
 */
typedef struct
{
	/**
	 * @brief	Коэффициент коррекции ошибки положения;
	 * @warning Это поле должно быть определено при инициализации стуктуры;
	 */
	float c1;

	/**
	 * @brief	Коэффициент коррекции ошибки скорости;
	 * @warning	Это поле должно быть определено при инициализации стуктуры;
	 */
	float c2;

	/**
	 * @brief	Коэффициент коррекции интегральной составляющей ошибки положения;
	 * @warning	Это поле должно быть определено при инициализации стуктуры;
	 */
	float lambda;

	/**
	 * @brief	Интегральная состоавляющая от ошибки положения;
	 */
	float chi;

	/**
	 * @brief	Желаемая угловая скорость;
	 */
	float omega_xd;

	/**
	 * @brief	Производная от желаемого положения;
	 */
	float phi_d_deriv;

	/**
	 * @brief	Желаемое положение на шаге <t-1>;
	 */
	float phi_d_t1;

	/**
	 * @brief	Величина шага для нахождения производной;
	 */
	float dT;

	/**
	 * @brief 	Коэффициент;
	 * @warning	Это поле должно быть определено при инициализации стуктуры;
	 * @see		eq. 4.53;
	 */
	float b1;

	/**
	 * @brief	Степерь, в которую необходимо возвести ошибку "e1";
	 */
	float e1PowCoeff;

	/**
	 * @brief	Степень, в которую необходимо возвести ошибку "e2"
	 */
	float e2PowCoeff;

	/**
	 * @brief	Максимальное значение по модулю выходного параметра;
	 */
	float saturation;

	/**
	 * @brief	Переключатели режима работы регулятора;
	 */
	struct
	{
		/**
		 * @brief	Взятие ошибки "e1" по модулю;
		 */
		size_t e1TakeModule;

		/**
		 * @brief	Интерпретация входного параметра "phi_d" как "e1";
		 */
		size_t phi_d_its_e1;

	} tumblers; /*	Переключатели режима работы регулятора */

} REGUL_integ_back_step_s;

typedef struct
{
	float kp;

	float ki;

	float kd;

	/**
	 * @brief	Интегральная состоавляющая ошибки;
	 */
	float integral;

	/**
	 * @brief	Интегральная состоавляющая ошибки после коррекции насыщения;
	 */
	float integralAfterCorrect;

	/**
	 * @brief	Ошибка за предыдущий момент времени;
	 */
	float error_t1;

	/**
	 * @brief	Величина шага для нахождения производной;
	 */
	float dT;

	/**
	 * @brief	Ограничение насыщения;
	 */
	float saturation;
} REGUL_pid_regulator_s;
/******************************************************************************/


/******************************************************************************/
//  Секция определения глобальных переменных
#if	defined __REGUL_REGULATORS_DEBUG__
extern float g_e1;
extern float g_e2;
extern float g_IntegralBackStepReturnValue;
extern float g_omega_x;
extern float g_chi;
extern float g_b1;
#endif
/******************************************************************************/


/******************************************************************************/
//  Секция прототипов глобальных функций
extern float REGUL_IntegralBackStep(
                                    REGUL_integ_back_step_s *pStruct,
                                    float phi_d,
                                    float phi,
                                    float omega_x);

extern void REGUL_Init_IntergralBackStep(
                                         REGUL_integ_back_step_s *pStruct);

extern float REGUL_PI_regulator(
                                REGUL_pid_regulator_s *pSturct,
                                float error);
/******************************************************************************/


/******************************************************************************/
//  Секция определения макросов
/******************************************************************************/
#endif /* LIB_A_REGUL_REGULATORS_H_ */

////////////////////////////////////////////////////////////////////////////////
//  END OF FILE
////////////////////////////////////////////////////////////////////////////////
