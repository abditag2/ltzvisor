/********************************* Copyright ********************************
 **
 ** Copyright (c) 2015,
 ** Aerospace Information Technology (AIT), Universitat Wurzburg
 ** Embedded Systems Research Group (ESRG), Universidade do Minho
 ** All rights reserved.
 **
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **
 ** 1 Redistributions of source code must retain the above copyright
 **   notice, this list of conditions and the following disclaimer.
 **
 ** 2 Redistributions in binary form must reproduce the above copyright
 **   notice, this list of conditions and the following disclaimer in the
 **   documentation and/or other materials provided with the
 **   distribution.
 **
 ** 3 Neither the name of the Aerospace Information Technology and
 **   Embedded Systems Research Group, nor the names of its contributors
 **   may be used to endorse or promote products derived from this software
 **   without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **
 ****************************************************************************/

/**
* @file zynq_ttc.c
* @date 2015/10/03 16:00
* @author Sandro Pinto
*
* Copyright 2015 AIT & ESRG
*
* @brief Zynq Triple Timer Counter Driver
*/

#ifndef __ZYNQ_TTC_H_
#define __ZYNQ_TTC_H_


#include"types.h"
#include"platform.h"
#include"gic.h"

#define TTC_FREQ					(CLOCK_FREQUENCY/6)

/* Triple Timer Counter Interrupts IDs */
#define TTC0_TTCx_1_INTERRUPT		42
#define TTC0_TTCx_2_INTERRUPT		43
#define TTC0_TTCx_3_INTERRUPT		44
#define TTC1_TTCx_1_INTERRUPT		69
#define TTC1_TTCx_2_INTERRUPT		70
#define TTC1_TTCx_3_INTERRUPT		71

/* Triple Timer Counter - Clock Control Register Flags */
#define TTC_CLK_CNTRL_EXT_EDGE    (1 << 6)        // External Clock Edge
#define TTC_CLK_CNTRL_SRC         (1 << 5)        // Clock Source
#define TTC_CLK_CNTRL_PS_EN       (1 << 0)        // Prescale enable
#define TTC_CLK_CNTRL_PS_VAL      (4)          	  // Prescale value

/* Triple Timer Counter - Counter Control Register Flags */
#define TTC_CNT_CNTRL_POL_WAVE    (1 << 6)        // Waveform polarity
#define TTC_CNT_CNTRL_EN_WAVE     (1 << 5)        // Output waveform enable
#define TTC_CNT_CNTRL_RST         (1 << 4)        // Reset counter value and restarts counting
#define TTC_CNT_CNTRL_MATCH       (1 << 3)        // Register Match mode
#define TTC_CNT_CNTRL_DEC         (1 << 2)        // Decrement
#define TTC_CNT_CNTRL_INT         (1 << 1)        // Interval Mode
#define TTC_CNT_CNTRL_DIS         (1 << 0)        // Disable Counter

/* Triple Timer Counter - Interrupt Enable Register Flags */
#define TTC_INT_EN_EV             (1 << 5)        // Event timer overflow interrupt
#define TTC_INT_EN_CNT_OVR        (1 << 4)        // Counter overflow
#define TTC_INT_EN_MATCH3         (1 << 3)        // Match 3 interrupt
#define TTC_INT_EN_MATCH2         (1 << 2)        // Match 2 interrupt
#define TTC_INT_EN_MATCH1         (1 << 1)        // Match 1 interrupt
#define TTC_INT_EN_INTERVAL       (1 << 0)        // Interval interrupt

/* Triple Timer Counter - Event Control Timer Register Flags */
#define TTC_EV_CNTRL_TIM_E_OV     (1 << 2)        // External Clock Edge
#define TTC_EV_CNTRL_TIM_E_LO     (1 << 1)        // Clock Source
#define TTC_EV_CNTRL_TIM_E_EN     (1 << 0)        // Enable Event timer

/* Triple Timer Counter - Reset Configuration value */
#define TTC_RESET_CONFIG          (0x21)          // Output waveform enable & disable counter

/* Triple Timer Counter - Static Interval Value */
#define TTC_INT_VALUE             (0.000001*TTC_FREQ)          // No Prescale -> F = CPU_FREQ/6 -> Period = 1/F -> (1us = 1*0.000001*F ticks)
/* Triple Timer Counter - Static Match1 Value */
#define TTC_MATCH1_VALUE          (0.000001*TTC_FREQ)          // No Prescale -> F = CPU_FREQ/6 -> Period = 1/F -> (1us = 1*0.000001*F ticks)

/* Number of TTCs */
#define NUM_TTC    2
/* Number of Timers per TTC */
#define NUM_TIM_PER_TTC    3

/* TTC Number Definition */
#define TTC0        0
#define TTC1        1
/* TTC's Timer Number Definition */
#define TTCx_1      0
#define TTCx_2      1
#define TTCx_3      2

/* TTC Modes */
#define INTERVAL       0
#define MATCH          1
#define FREE_RUNNING   2


/* TTC registers structure */
typedef struct
{
	volatile uint32_t clk_cntrl[NUM_TIM_PER_TTC];              // Clock Control Register
	volatile uint32_t cnt_cntrl[NUM_TIM_PER_TTC];              // Operational mode and reset
	volatile const uint32_t cnt_value[NUM_TIM_PER_TTC];        // Current counter value
	volatile uint32_t interv_cnt[NUM_TIM_PER_TTC];             // Interval value
	volatile uint32_t match1_cnt[NUM_TIM_PER_TTC];             // Match value
	volatile uint32_t match2_cnt[NUM_TIM_PER_TTC];             // Match value
	volatile uint32_t match3_cnt[NUM_TIM_PER_TTC];             // Match value
	volatile const uint32_t interrupt_reg[NUM_TIM_PER_TTC];    // Counter Interval, Match, Overflow and Event interrupts (RO)
	volatile uint32_t interrupt_en[NUM_TIM_PER_TTC];           // ANDed with corresponding Interrupt Register
	volatile uint32_t event_cntrl_tim[NUM_TIM_PER_TTC];        // Enable, pulse and overflow
	volatile const uint32_t event_reg[NUM_TIM_PER_TTC];        // pclk cycle count for event
} Zynq_Ttc;


typedef enum
{
	Interval = 0x00,
	Match = 0x01,
	Free_Running = 0x02,
	Disable = 0xff
}TTC_Mode_TypeDef;


/**********************************************************************************
 * TTC init
 * Initializes one timer of one TTC (There are 2 TTC and 3 timers per TTC)
 * @param   ttc_num (TTC number), timer_num (TTC's timer number), mode (The timer mode)
 * @retval  Error code
 **********************************************************************************/
uint32_t ttc_init(uint32_t ttc_num, uint32_t timer_num, TTC_Mode_TypeDef mode);


/**********************************************************************************
 * TTC enable
 * Enable one timer of one TTC (There are 2 TTC and 3 timers per TTC)
 * @param    ttc_num (TTC number), timer_num (TTC's timer number)
 * @retval   Error code
 **********************************************************************************/
uint32_t ttc_enable(uint32_t ttc_num, uint32_t timer_num);


/**********************************************************************************
 * TTC disable
 * Disable one timer of one TTC (There are 2 TTC and 3 timers per TTC)
 * @param    ttc_num (TTC number), timer_num (TTC's timer number)
 * @retval   Error code
 **********************************************************************************/
uint32_t ttc_disable(uint32_t ttc_num, uint32_t timer_num);


/**********************************************************************************
 * Set counter value
 * Set counter value for a specific TTC timer
 * @param    ttc_num (TTC number), timer_num (TTC's timer number), value (counter)
 * @retval   Error code
 **********************************************************************************/
uint32_t ttc_value_set(uint32_t ttc_num, uint32_t timer_num, uint32_t value);


/**********************************************************************************
 * Get counter value
 * Get counter value for a specific TTC timer
 * @param    ttc_num (TTC number), timer_num (TTC's timer number)
 * @retval   value (counter)
 **********************************************************************************/
uint32_t ttc_value_get(uint32_t ttc_num, uint32_t timer_num);


uint32_t ttc_usec_get(uint32_t ttc_num, uint32_t timer_num);


/**********************************************************************************
 * TTC timer request
 * Request a timer handler to be called after a number of cycles.
 * @param    Xxxxxx
 * @retval   No return value
 **********************************************************************************/
uint32_t ttc_request(uint32_t ttc_num, uint32_t timer_num, uint32_t value);


/**********************************************************************************
 * TTC interrupt clear
 * Request a timer handler to be called after a number of microseconds.
 * @param    Xxxxxx
 * @retval   No return value
 **********************************************************************************/
uint32_t ttc_interrupt_clear(uint32_t irq_id);


uint32_t ttc_periodns_get(uint32_t ttc_num, uint32_t timer_num);



#endif /* __ZYNQ_TTC_H_ */
