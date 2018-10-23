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
#include "../Lib_A_DIFF_differentiators/Lib_A_DIFF_differentiators.h"
#include "../Lib_A_NINTEG_numerical_integration/Lib_A_NINTEG_numerical_integration.h"
/*============================================================================*/
/******************************************************************************/


/******************************************************************************/
//  Секция определения констант
#if !defined (__REGUL_FPT__)
#error "Please, set __REGUL_FPT__ (default value is float)"
#endif

#define REGUL_EN                                    1
#define REGUL_DIS                                   0
/******************************************************************************/


/******************************************************************************/
//  Секция определения типов

typedef enum
{
	REGUL_ERROR = 0,
	REGUL_SUCCESS,
} regul_fnc_status_e;

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
	__REGUL_FPT__ c1;

	/**
	 * @brief Коэффициент коррекции ошибки скорости;
	 * @warning   Это поле должно быть определено при инициализации стуктуры;
	 */
	__REGUL_FPT__ c2;

	/**
	 * @brief     Коэффициент;
	 * @warning   Это поле должно быть определено при инициализации стуктуры;
	 * @see       eq. 4.53;
	 */
	__REGUL_FPT__ b1;

	/**
	 * @brief Коэффициент для второй производной от e1
	 */
	__REGUL_FPT__ e1SecondDerivCoeff;
	/*---- |End  | <-- Коэффициенты регулятора -------------------------------*/

	/**
	 * @brief Коэффициент коррекции интегральной составляющей ошибки положения;
	 * @warning   Это поле должно быть определено при инициализации стуктуры;
	 */
	__REGUL_FPT__ lambda;

	/**
	 * @brief Интегральная состоавляющая от ошибки положения;
	 */
	__REGUL_FPT__ chi;

	/**
	 * @brief Желаемая угловая скорость;
	 */
	__REGUL_FPT__ omega_xd;

	/**
	 * @brief Производная от желаемого положения;
	 */
	__REGUL_FPT__ phi_d_deriv;

	/**
	 * @brief Желаемое положение на шаге <t-1>;
	 */
	__REGUL_FPT__ phi_d_t1;

	/**
	 * @brief Величина шага для нахождения производной;
	 */
	__REGUL_FPT__ dT;

	/**
	 * @brief Степерь, в которую необходимо возвести ошибку "e1";
	 */
	__REGUL_FPT__ e1PowCoeff;

	/**
	 * @brief Степень, в которую необходимо возвести ошибку "e2"
	 */
	__REGUL_FPT__ e2PowCoeff;

	/**
	 * @brief Максимальное значение по модулю выходного параметра регулятора;
	 */
	__REGUL_FPT__ saturation;

	/**
	 * @brief Структура для дифференцирования переменной phi_d
	 */
	diff_differentiator_1_s phi_d_derivStruct;

	/*
	 * @brief Структура для дифференцирования e1
	 */
	diff_differentiator_1_s e1_FirstDerivStruct,
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
	struct
	{
		/**
		 * @brief Коэффициент усиления для пропорциональной составляющей ошибки
		 */
		__REGUL_FPT__ kP;
	} proportional_s;

	struct
	{
		/**
		 * @brief	Коэффициент усиления для интегральной составляющей ошибки
		 */
		__REGUL_FPT__ kI;

		/**
		 * @brief 	Значение интегральной составляющей ошибки
		 */
		__REGUL_FPT__ val;

		/**
		 * @brief 	Значение насыщения интегральной составляющей регулятора
		 */
		__REGUL_FPT__ satur;

		/**
		 * @brief	Структура для интегрирования методом трапеций. Необходима
		 *        	для нахождения интегральной составляющей регулятора
		 */
		ninteg_trapz_s deltaTrap_s;
	} integral_s;

	struct
	{
		/**
		 * @brief	Коэффициент усиления дифференциальной составляющей ошибки
		 */
		__REGUL_FPT__ kD;

		/**
		 * @brief	Стук тура для дифференцирования методом 1-го порядка
		 */
		diff_differentiator_1_s different1_s;
	} derivative_s;

	/**
	 * @brief 	Величина шага для нахождения производной и нахождения
	 *         	интегральной составляющей методом трапеций
	 */
	__REGUL_FPT__ dT;

	/**
	 * @brief 	Значение насыщения возвращаемого регулятором значения
	 */
	__REGUL_FPT__ pidValSatur;

	/**
	 * @brief 	Статус инициализации структуры
	 */
	regul_fnc_status_e initStatus_e;
} regul_pid_s;

typedef struct
{
	/**
	 * @brief	Период интегрирования/дифференцирования
	 */
	__REGUL_FPT__ dT;

	/**
	 * @brief	Коэффициент пропорциональной составляющей регулятора
	 */
	__REGUL_FPT__ kP;

	/**
	 * @brief	Коэффициент интегральной составляющей регулятора
	 */
	__REGUL_FPT__ kI;

	/**
	 * @brief	Коэффициент дифференциальной составляющей регулятора
	 */
	__REGUL_FPT__ kD;

	/**
	 * @brief	Значение насыщения интегральной составляющей регулятора
	 */
	__REGUL_FPT__ integralValSaturation;

	/**
	 * @brief	Значение насыщения выходной величины ПИД регулятора
	 */
	__REGUL_FPT__ returnValSaturation;
} regul_pid_init_struct_s;
/******************************************************************************/


/******************************************************************************/
//  Секция определения глобальных переменных
#if defined __REGUL_REGULATORS_DEBUG__
extern __REGUL_FPT__ g_e1;
extern __REGUL_FPT__ g_e2;
extern __REGUL_FPT__ g_IntegralBackStepReturnValue;
extern __REGUL_FPT__ g_omega_x;
extern __REGUL_FPT__ g_omega_xd;
extern __REGUL_FPT__ g_phi_d_deriv;
extern __REGUL_FPT__ g_chi;
extern __REGUL_FPT__ g_b1;
#endif
/*******F***********************************************************************/


/******************************************************************************/
//  Секция прототипов глобальных функций
extern float
REGUL_Get_IBSC(
	regul_ibsc_s *pStruct,
	__REGUL_FPT__ phi_d,
	__REGUL_FPT__ phi,
	__REGUL_FPT__ omega_x);

extern void
REGUL_Init_IBSC(
	regul_ibsc_s *pStruct);

extern __REGUL_FPT__
REGUL_Get_PID(
	regul_pid_s *pPID_s,
	__REGUL_FPT__ err,
	__REGUL_FPT__ errDeriv);

//extern void
//REGUL_Init_PID (
//	regul_pid_s *pDID_s,
//	__REGUL_FPT__ kP,
//	__REGUL_FPT__ kI,
//	__REGUL_FPT__ kD,
//	__REGUL_FPT__ dT,
//	__REGUL_FPT__ returnValSaturation,
//	__REGUL_FPT__ integValSaturation);

extern regul_fnc_status_e
REGUL_Init_PID(
	regul_pid_s *p_s,
	regul_pid_init_struct_s *pInit_s);

extern void
REGUL_PID_StructInit(
	regul_pid_init_struct_s *pInitStruct);

extern __REGUL_FPT__
REGUL_MixTwoVal(
	__REGUL_FPT__ firstVal,
	__REGUL_FPT__ secondVal,
	__REGUL_FPT__ mixCoeff);

extern __REGUL_FPT__
REGUL_PowerFunc (
	__REGUL_FPT__ basis,
	__REGUL_FPT__ exponent);
/******************************************************************************/


/******************************************************************************/
//  Секция определения макросов
/******************************************************************************/
#endif /* LIB_A_REGUL_REGULATORS_H_ */

////////////////////////////////////////////////////////////////////////////////
//  END OF FILE
////////////////////////////////////////////////////////////////////////////////
