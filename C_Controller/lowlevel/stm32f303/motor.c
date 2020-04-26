#include "recovid_revB.h"
#include "lowlevel.h"
#include "sensing.h"
#include "ihm_communication.h"
#include "hardware_serial.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dma.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

static TIM_HandleTypeDef* _motor_tim = NULL;

static void period_elapsed_callback(TIM_HandleTypeDef *tim);

static volatile bool _moving;
static volatile bool _homing;
static volatile bool _home;
static volatile bool _limit_sw_A;
static volatile bool _limit_sw_B;
static volatile bool _active;

static void check_home() ;
static void motor_enable(bool ena);


uint16_t compute_motor_press_constant(uint16_t step_t_us, uint16_t nb_steps, uint16_t* steps_t_us)
{
	const uint16_t max_steps = MIN(nb_steps, MOTOR_MAX);
	const uint16_t deceleration_step = 100;
	for(unsigned int i = 0; i < max_steps; i++)
	{
		steps_t_us[i] = step_t_us;
		////Acceleration phase
		//if(i < max_steps) {
		//	steps_t_us[i] = MAX(step_t_us, MOTOR_STEP_TIME_INIT - (A)*i);
		//}
		////Deceleration phase
		//else if(max_steps > deceleration_step && i < max_steps-deceleration_step) {
		//	steps_t_us[i] = MAX(step_t_us, step_t_us + (A)*(max_steps-i));
		//}
		//else {
		//	steps_t_us[i] = UINT16_MAX;
		//}
	}
	return max_steps;
}

int motor_press_dir(uint16_t* steps_profile_us, uint16_t nb_steps, int direction)
{
    motor_stop();
	if (nb_steps > 0) {
		HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, direction);
		if(direction == MOTOR_RELEASE_DIR) {
			light_green(On);
			light_red(Off);
		}
		else {
			light_green(Off);
			light_red(On);
		}
		_moving=true;
		_motor_tim->Instance->ARR = steps_profile_us[0];
		HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
		if(direction == MOTOR_PRESS_DIR)
			HAL_TIM_DMABurst_MultiWriteStart(_motor_tim, TIM_DMABASE_ARR, TIM_DMA_UPDATE,  (uint32_t*)&steps_profile_us[1], TIM_DMABURSTLENGTH_1TRANSFER, nb_steps-1);
	}
    return nb_steps;
}

int motor_press(uint16_t* steps_profile_us, uint16_t nb_steps)
{
  return motor_press_dir(steps_profile_us, nb_steps, MOTOR_PRESS_DIR);
}



int motor_release() {
  int nb_steps = 0;
  if(!_home) {
    motor_stop();
    nb_steps = compute_motor_press_constant(300, MOTOR_MAX, steps_release_t_us);
    _moving=true;
    _homing=true;
    motor_press_dir(steps_release_t_us, nb_steps, MOTOR_RELEASE_DIR);
  }
  return nb_steps;
}

uint16_t motor_press_constant(uint16_t step_t_us, uint16_t nb_steps)
{
	uint16_t max_steps = compute_motor_press_constant(step_t_us, nb_steps, steps_press_t_us);
    motor_press(steps_press_t_us, max_steps);
    return max_steps;
}

bool motor_stop() 
{
  HAL_TIM_PWM_Stop(_motor_tim, MOTOR_TIM_CHANNEL);
  HAL_TIM_DMABurst_WriteStop(_motor_tim, TIM_DMA_ID_UPDATE);
  HAL_TIM_Base_Init(&htim2);
  HAL_DMA_Init(&hdma_tim2_up);
  _moving=false;
  return true;
}

bool is_motor_ok() {
  return _motor_tim!=NULL;
}

bool init_motor()
{
  if(_motor_tim==NULL) {
    _motor_tim= &motor_tim;
    // register IT callbacks
    _motor_tim->PeriodElapsedCallback = period_elapsed_callback;

    _active= HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);

    _limit_sw_A= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    _limit_sw_B= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    check_home();

    motor_enable(true);

    if(_home) {
      HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
      _motor_tim->Instance->ARR = MOTOR_HOME_STEP_US;
      _moving = true;
      _homing= false;
      HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);    
      while(_home);
      HAL_Delay(200);
      motor_stop();
    }
    _homing=true;
    _moving=true;
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
    _motor_tim->Instance->ARR = MOTOR_HOME_STEP_US;
    HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);    
    while(!_home);
  }
  return true;
}

static void period_elapsed_callback(TIM_HandleTypeDef *tim)
{
  motor_stop();
}

void motor_limit_sw_A_irq() {
  static uint32_t last_time=0;
  uint32_t time= HAL_GetTick();
  if(time-last_time>100) {
    _limit_sw_A= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    check_home();
  }
  last_time=time;
}

void motor_limit_sw_B_irq() {  
  static uint32_t last_time=0;
  uint32_t time= HAL_GetTick();
  if(time-last_time>100) {
    _limit_sw_B= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    check_home();
  }
  last_time=time;
}

void motor_active_irq() {
  static uint32_t last_time=0;
  uint32_t time= HAL_GetTick();
  if(time-last_time>50) {
    _active=HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);
  }
  last_time=time;
}


static void check_home() {
  if(_limit_sw_A && _limit_sw_B) {
    _home=true;
    if(_homing) {
      motor_stop();
      _homing= false;
    }
  } else {
    _home=false;
  }
}

static void motor_enable(bool ena) {
	HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, ena?GPIO_PIN_SET:GPIO_PIN_RESET);
}






static void print_samples_P(uint16_t* samples_P, uint16_t* samples_P_dt_us, int nb_samples)
{
	static char msg[20];
	strcpy(msg, "\nP");
	hardware_serial_write_data(msg, strlen(msg)); 
	msg[1]='\n';
	for(unsigned int j=0; j < nb_samples; j++)
	{
		char* current = itoa( (int) samples_P[j], msg+1, 10);
		char* next = current + strlen(current);
		next[0] = ' ';
		next++;
		itoa( (int) samples_P_dt_us[j], next, 10);
		hardware_serial_write_data(msg, strlen(msg)); 
		wait_ms(1);
	}}

static void print_samples_Q(float* samples_Q_Lps, uint16_t* samples_Q_Lps_dt_us, int nb_samples)
{
	static char msg[200];
	strcpy(msg, "\nQ");
	hardware_serial_write_data(msg, strlen(msg)); 
	msg[1]='\n';
	for(unsigned int j=0; j < nb_samples; j++)
	{
		char* current = itoa( (int) (samples_Q_Lps[j] * 1000.0f), msg+1, 10);
		char* next = current + strlen(current);
		next[0] = ' ';
		next++;
		itoa( (int) samples_Q_Lps_dt_us[j], next, 10);
		hardware_serial_write_data(msg, strlen(msg)); 
		wait_ms(1);
	}
}


static void print_steps(uint16_t* steps_t_us, unsigned int nb_steps)
{
	static char msg[200];
	strcpy(msg, "\nSteps                ");
	itoa( nb_steps , msg+6, 10);
	hardware_serial_write_data(msg, strlen(msg)); 
	msg[0] = '\n';
	for(unsigned int j=0; j < nb_steps; j+=100)
	{
		itoa((int) steps_t_us[j], msg+1, 10);
		hardware_serial_write_data(msg, strlen(msg)); 
		wait_ms(1);
	}
}

void test_motor() 
{
	int nb_steps;
	nb_steps = motor_release();
	while(!_home) {};
	wait_ms(2000);
	while(true) 
	{
		motor_stop();
		valve_inhale();
		sensors_start_sampling_flow();
		nb_steps = motor_press_constant(400, 3000);
		wait_ms(1000);
		sensors_stop_sampling_flow();
		nb_steps = motor_release();
		wait_ms(10);
		while(!_home) {};
		valve_exhale();
		print_samples_P(samples_P, samples_P_dt_us, get_samples_P_index_size());
		//print_samples_Q(samples_Q_Lps, samples_Q_Lps_dt_us, get_samples_Q_index_size());
		//print_steps(steps_press_t_us, nb_steps);
		wait_ms(2000);
	}
}
