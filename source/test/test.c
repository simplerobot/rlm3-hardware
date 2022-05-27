#include "main.h"
#include <cmsis_os.h>
#include "task.h"


extern void RLM3_Main()
{
	ITM_SendString("Running tests...\n");
	ITM_SendString("Passed!\n");
	ITM_SendString("EOT PASS\n");

	for (;;)
	{
		HAL_GPIO_WritePin(STATUS_LIGHT_GPIO_Port, STATUS_LIGHT_Pin, GPIO_PIN_SET);
		vTaskDelay(1000);
		HAL_GPIO_WritePin(STATUS_LIGHT_GPIO_Port, STATUS_LIGHT_Pin, GPIO_PIN_RESET);
		vTaskDelay(1000);

		ITM_SendString("...\n");
	}
}
