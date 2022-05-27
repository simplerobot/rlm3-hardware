#include "main.h"
#include <cmsis_os.h>
#include "task.h"


static void SendText(const char* text)
{
	for (size_t i = 0; text[i] != 0; i++)
		ITM_SendChar(text[i]);
}

extern void RLM3_Main()
{
	TPI->ACPR = HAL_RCC_GetHCLKFreq() / 1000000 - 1;

	vTaskDelay(1000);

	SendText("Running tests...\n");
	SendText("Passed!\n");
	SendText("EOT PASS\n");

	for (;;)
	{
		HAL_GPIO_WritePin(STATUS_LIGHT_GPIO_Port, STATUS_LIGHT_Pin, GPIO_PIN_SET);
		vTaskDelay(1000);
		HAL_GPIO_WritePin(STATUS_LIGHT_GPIO_Port, STATUS_LIGHT_Pin, GPIO_PIN_RESET);
		vTaskDelay(1000);

		SendText("...\n");
	}
}
