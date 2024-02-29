/*
 * msp.c
 *
 *  Created on: Jan 25, 2024
 *      Author: wardawg
 */


#include <main.h>

void HAL_MspInit(void) {
	//Here,we do the Low Level Processor Initializations.

	//1. Set up the priority grouping of Processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2. Enable the required exceptions (system) of the Arm Cortex Mx Processor
	SCB->SHCSR |= 0x7 << 16; //For setting the usage fault, bus fault and memory fault system exceptions

	//3. Configure the priority for the system initializations.
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

	/**CAN1 GPIO Configuration
	PD0     ------> CAN1_RX
	PD1     ------> CAN1_TX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
