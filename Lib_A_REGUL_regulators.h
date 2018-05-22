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
#include    "../Lib_A_DIFF_differentiators/Lib_A_DIFF_differentiators.h"
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
//  Секция определения констант
#if !defined (REGUL_FLOAT_POINT_TYPE)
#define REGUL_FLOAT_POINT_TYPE    float
#endif

#define REGUL_EN                                    1
#define REGUL_DIS                                   0
/******************************************************************************/


/******************************************************************************/
//  Секция определения типов

/**
 * @brief   Структура, содержащая переменные для работы IBSC регулятора;
 */
typedef struct
{
	/*---- |Begin| --> Коэффициенты регулятора -------------------------------*/
	/**
	 * @brief Коэффициент коррекции ошибки положения;
	 * @warning Это поле должно быть определено при инициализации стуктуры;
	 */
	float c1;

	/**
	 * @brief Коэффициент коррекции ошибки скорости;
	 * @warning   Это поле должно быть определено при инициализации стуктуры;
	 */
	float c2;

	/**
	 * @brief     Коэффициент;
	 * @warning   Это поле должно быть определено при инициализации стуктуры;
	 * @see       eq. 4.53;
	 */
	float b1;

	/**
	 * @brief Коэффициент для второй производной от e1
	 */
	float e1SecondDerivCoeff;
	/*---- |End  | <-- Коэффициенты регулятора -------------------------------*/

	/**
	 * @brief Коэффициент коррекции интегральной составляющей ошибки положения;
	 * @warning   Это поле должно быть определено при инициализации стуктуры;
	 */
	float lambda;

	/**
	 * @brief Интегральная состоавляющая от ошибки положения;
	 */
	float chi;

	/**
	 * @brief Желаемая угловая скорость;
	 */
	float omega_xd;

	/**
	 * @brief Производная от желаемого положения;
	 */
	float phi_d_deriv;

	/**
	 * @brief Желаемое положение на шаге <t-1>;
	 */
	float phi_d_t1;

	/**
	 * @brief Величина шага для нахождения производной;
	 */
	float dT;

	/**
	 * @brief Степерь, в которую необходимо возвести ошибку "e1";
	 */
	float e1PowCoeff;

	/**
	 * @brief Степень, в которую необходимо возвести ошибку "e2"
	 */
	float e2PowCoeff;

	/**
	 * @brief Максимальное значение по модулю выходного параметра регулятора;
	 */
	float saturation;

	/**
	 * @brief Структура для дифференцирования переменной phi_d
	 */
	DIFF_differentiator_1_s phi_d_derivStruct;

	/*
	 * @brief Структура для дифференцирования e1
	 */
	DIFF_differentiator_1_s e1_FirstDerivStruct,
							e1_SecontDerivStruct;

	/**
	 * @brief Переключатели режима работы регулятора;
	 */
	struct {
		/**
		 * @brief   Взятие ошибки "e1" по модулю;
		 */
		size_t e1TakeModuleFlag;

		/**
		 * @brief   Интерпретация входного параметра "phi_d" как "e1"
		 * @see     brief функции REGUL_IntegralBackStep()
		 * @note    Если phi_d_its_e1Flag == 1, то параметр "phi" в функции
		 *          REGUL_IntegralBackStep() не используется
		 */
		size_t phi_d_its_e1Flag;

		/**
		 * @brief   Флаг разрешает возведение в степень переменных e1, e2;
		 */
		size_t enablePowFunctFlag;

	} tumblers; /*    Переключатели режима работы регулятора */

} regul_ibsc_s;

typedef struct
{
	REGUL_FLOAT_POINT_TYPE kP;

	REGUL_FLOAT_POINT_TYPE kI;

	REGUL_FLOAT_POINT_TYPE kD;

	/**
	 * @brief Интегральная составляющая ошибки
	 */
	REGUL_FLOAT_POINT_TYPE integVal;

	/**
	 * @brief Величина шага для нахождения производной
	 */
	REGUL_FLOAT_POINT_TYPE dT;

	/**
	 * @brief Значение насыщения возвращаемого регулятором значения
	 */
	REGUL_FLOAT_POINT_TYPE returnValSaturation;

	/**
	 * @brief Значение насыщения интегральной составляющей регулятора
	 */
	REGUL_FLOAT_POINT_TYPE integValSaturation;
} regul_pid_s;
/******************************************************************************/


/******************************************************************************/
//  Секция определения глобальных переменных
#if defined __REGUL_REGULATORS_DEBUG__
extern float g_e1;
extern float g_e2;
extern float g_IntegralBackStepReturnValue;
extern float g_omega_x;
extern float g_omega_xd;
extern float g_phi_d_deriv;
extern float g_chi;
extern float g_b1;
#endif
/*******F***********************************************************************/


/******************************************************************************/
//  Секция прототипов глобальных функций
extern float
REGUL_Get_IBSC(
	regul_ibsc_s *pStruct,
	float phi_d,
	float phi,
	float omega_x);

extern void
REGUL_Init_IBSC(
	regul_ibsc_s *pStruct);

extern REGUL_FLOAT_POINT_TYPE
REGUL_Get_PID(
	regul_pid_s *pData_struct,
	REGUL_FLOAT_POINT_TYPE err,
	REGUL_FLOAT_POINT_TYPE derivErr);

extern void
REGUL_Init_PID (
	regul_pid_s *pDID_s,
	REGUL_FLOAT_POINT_TYPE kP,
	REGUL_FLOAT_POINT_TYPE kI,
	REGUL_FLOAT_POINT_TYPE kD,
	REGUL_FLOAT_POINT_TYPE dT,
	REGUL_FLOAT_POINT_TYPE returnValSaturation,
	REGUL_FLOAT_POINT_TYPE integValSaturation);

extern REGUL_FLOAT_POINT_TYPE
REGUL_MixTwoVal(
	REGUL_FLOAT_POINT_TYPE firstVal,
	REGUL_FLOAT_POINT_TYPE secondVal,
	REGUL_FLOAT_POINT_TYPE mixCoeff);

extern float
REGUL_PowerFunc (
	float basis,
	float exponent);
/******************************************************************************/


/******************************************************************************/
//  Секция определения макросов
/******************************************************************************/
#endif /* LIB_A_REGUL_REGULATORS_H_ */

////////////////////////////////////////////////////////////////////////////////
//  END OF FILE
////////////////////////////////////////////////////////////////////////////////
