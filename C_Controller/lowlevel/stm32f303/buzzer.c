#include "recovid_revB.h"
#include "lowlevel.h"


bool buzzer_low(OnOff val) {
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool buzzer_medium(OnOff val) {
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool buzzer_high(OnOff val) {
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}
