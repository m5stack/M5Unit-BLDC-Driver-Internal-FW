/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "pid_controller.h"
#include <string.h>
#include "flash.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APPLICATION_ADDRESS     ((uint32_t)0x08001800)

#define MDIR_FFW 0
#define MDIR_REV 1

#define I2C_ADDRESS 0x65
#define FLASH_DATA_SIZE 24
#define FIRMWARE_VERSION 1

#define MAX_RECORD_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//捕获状�??
//[7]:0,没有成功的捕�??;1,成功捕获到一�??.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平�??.
//[5:0]:捕获低电平后溢出的次�??(对于32位定时器来说,1us计数器加1,溢出时间:4294�??)
uint8_t  TIM3CH1_CAPTURE_STA=0;	//输入捕获状�??		    				
uint32_t	TIM3CH1_CAPTURE_VAL;	//输入捕获�??(TIM2/TIM5�??32�??)

// 定义全局变量
float alpha = 0.9877f;
float y_prev = 0.0f;

volatile long long temp = 0;
volatile float setspeed, speed, rpm;
volatile float setting_rpm = 0;
static int16_t motor_throttle = 0;
volatile uint8_t boot_success = 0;
uint32_t reboot_delay = 0;

uint32_t config_pid[3] = {14200, 5536, 1580};
PIDControl pid_ctrl_pos_t;
int32_t out_data = 0;

uint8_t motor_mode = 1;
uint16_t motor_pwm = 0;
uint8_t pole_pairs = 7;

uint8_t motor_dir = 1;
uint8_t motor_model = 1;

uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;
uint16_t lowest_pwm = 10;

enum {MOTOR_STANBY = 0, MOTOR_RUNNING, MOTOR_ERROR};
volatile uint8_t motor_status = MOTOR_STANBY;
volatile uint32_t motor_boot_err_flag = 0;

volatile uint32_t flash_led_delay = 0;
volatile uint32_t flash_led_flag = 0;

float speed_record[MAX_RECORD_SIZE] = {0};
uint8_t record_index = 0;
volatile uint8_t avg_filter_level = 20;
volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);

	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

static void iap_gpio_init(void)
{  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
}

// 一阶低通滤波器函数
float lowPassFilter(float input) {
    float output = alpha * input + (1 - alpha) * y_prev;
    y_prev = output;
    return output;
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    flash_data[0] = i2c_address[0];
    flash_data[1] = 0;
    flash_data[2] = config_pid[0];
    flash_data[3] = (config_pid[0] >> 8);
    flash_data[4] = (config_pid[0] >> 16);
    flash_data[5] = (config_pid[0] >> 24);
    flash_data[6] = config_pid[1];
    flash_data[7] = (config_pid[1] >> 8);
    flash_data[8] = (config_pid[1] >> 16);
    flash_data[9] = (config_pid[1] >> 24);
    flash_data[10] = config_pid[2];
    flash_data[11] = (config_pid[2] >> 8);
    flash_data[12] = (config_pid[2] >> 16);
    flash_data[13] = (config_pid[2] >> 24);
    flash_data[18] = motor_model;
    flash_data[19] = pole_pairs;
    flash_data[20] = lowest_pwm;
    flash_data[21] = (lowest_pwm >> 8);
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    config_pid[0] = flash_data[2] | (flash_data[3] << 8) | (flash_data[4] << 16) | (flash_data[5] << 24);
    config_pid[1] = flash_data[6] | (flash_data[7] << 8) | (flash_data[8] << 16) | (flash_data[9] << 24);
    config_pid[2] = flash_data[10] | (flash_data[11] << 8) | (flash_data[12] << 16) | (flash_data[13] << 24);
    motor_model = flash_data[18];
    pole_pairs = flash_data[19];
    lowest_pwm = (flash_data[20] | (flash_data[21] << 8));
  }
}

void flash_data_write_back(void)
{
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[0] = i2c_address[0];
    flash_data[1] = 0;
    flash_data[2] = config_pid[0];
    flash_data[3] = (config_pid[0] >> 8);
    flash_data[4] = (config_pid[0] >> 16);
    flash_data[5] = (config_pid[0] >> 24);
    flash_data[6] = config_pid[1];
    flash_data[7] = (config_pid[1] >> 8);
    flash_data[8] = (config_pid[1] >> 16);
    flash_data[9] = (config_pid[1] >> 24);
    flash_data[10] = config_pid[2];
    flash_data[11] = (config_pid[2] >> 8);
    flash_data[12] = (config_pid[2] >> 16);
    flash_data[13] = (config_pid[2] >> 24);
    flash_data[18] = motor_model;
    flash_data[19] = pole_pairs;
    flash_data[20] = lowest_pwm;
    flash_data[21] = (lowest_pwm >> 8);
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  }     
}

//定时器输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调�??
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执�??
{
	if((TIM3CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
	{
		if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
                TIM3CH1_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);//获取当前的捕获�??.
                TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2);   //�??定要先清除原来的设置！！
                TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM5通道1上升沿捕�??
			}else  								//还未�??�??,第一次捕获上升沿
			{
				TIM3CH1_CAPTURE_STA=0;			//清空
				TIM3CH1_CAPTURE_VAL=0;
				TIM3CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升�??
				__HAL_TIM_DISABLE(&htim3);        //关闭定时�??5
				__HAL_TIM_SET_COUNTER(&htim3,0);
				TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2);   //�??定要先清除原来的设置！！
				TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//定时�??5通道1设置为下降沿捕获
				__HAL_TIM_ENABLE(&htim3);//使能定时�??5
			}		    
	}			
}

void reset_i2c_bus(void)
{
  HAL_I2C_DeInit(&hi2c2);  

	user_i2c_init();

  HAL_I2C_EnableListen_IT(&hi2c2);
}

void init_pid(void)
{
  PIDInit(&pid_ctrl_pos_t, 
      (float)config_pid[0]/100.0f, 
      (float)config_pid[1]/100.0f, 
      (float)config_pid[2]/100.0f, 0.001, 
      lowest_pwm, 2047, AUTOMATIC, DIRECT);  
  pid_ctrl_pos_t.setpoint = 0;
}

uint8_t is_boot_success(void)
{
  if (HAL_GPIO_ReadPin(DRV_RD_GPIO_Port, DRV_RD_Pin))
    return 1;
  else
    return 0;
}

void set_pwm(uint16_t duty)
{
  if (duty > 2047)
    duty = 2047;

  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, duty);
}

void SetMotorDir(uint8_t dir)
{
  if(dir)
  {
    HAL_GPIO_WritePin(GPIOA, DRV_FR_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, DRV_FR_Pin, GPIO_PIN_SET);
  }
}

void SetMotorPwm(uint16_t value)
{
  set_pwm(value);
}

void SetMotorThrottle(int16_t throttle)
{
    if(throttle>0)
    {
        SetMotorDir(MDIR_FFW);
        SetMotorPwm(throttle);
    }
    else
    {
        SetMotorDir(MDIR_REV);
        SetMotorPwm(throttle);   
    }
}

void speed_pid(void)
{
  static uint32_t next_update_time_pos_switch_mode = 0;
  
  pid_ctrl_pos_t.input = speed;
  if(next_update_time_pos_switch_mode < HAL_GetTick())
  {
    if (pid_ctrl_pos_t.setpoint == 0) {
      set_pwm(0);
      motor_status = MOTOR_STANBY;
    }
    else {
      // if (speed <= (pid_ctrl_pos_t.setpoint - 1) || speed >= (pid_ctrl_pos_t.setpoint + 1)) {
      //   next_update_time_pos_switch_mode = HAL_GetTick() + 1;
      //   PIDCompute(&pid_ctrl_pos_t);
      //   out_data = pid_ctrl_pos_t.output;
      //   if (motor_mode)
      //     set_pwm(out_data);
      //   else
      //     set_pwm(0);
      // }
      next_update_time_pos_switch_mode = HAL_GetTick() + 1;
      PIDCompute(&pid_ctrl_pos_t);
      out_data = pid_ctrl_pos_t.output;
      if (motor_mode)
        set_pwm(out_data);
      else
        set_pwm(0);      
      motor_status = MOTOR_RUNNING;
    }
  }
}

float avg_filter(float *data, int len)
{
	float sum = 0;
	float min = data[0];
	float max = data[0];
	for (int i = 0; i < len; i++) {
		if (data[i] < min) {
			min = data[i];
		}
		if (data[i] > max) {
			max = data[i];
		}
    sum += data[i];
	}

	sum -= min;
	sum -= max;

	return sum / (len - 2);
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t buf[4];
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  uint8_t rx_mark[16] = {0};  

  if (len > 1) {
    if (rx_data[0] == 0) {
      if (rx_data[1]) {
        motor_mode = 1;
        set_pwm(0);
        motor_pwm = 0;
      }
      else {
        motor_mode = 0;
        init_pid();
        set_pwm(0);
        setting_rpm = 0;
      }
    }
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x11) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x10+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x10+i] = 1;     
      } 
      if (rx_mark[0]) {
        motor_pwm = rx_buf[0];
      }      
      if (rx_mark[1]) {
        motor_pwm &= 0x00ff;
        motor_pwm |= (rx_buf[1] << 8);
      }
      set_pwm(motor_pwm);
    }
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x40+i] = 1;     
      } 
      if (rx_mark[0] && rx_mark[1] && rx_mark[2] && rx_mark[3]) {
        memcpy((uint8_t *)&setting_rpm, rx_buf, 4);
        pid_ctrl_pos_t.setpoint = setting_rpm * pole_pairs / 60;
      }      
    }
    else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x5B) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x50+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x50+i] = 1;     
      } 
      if (rx_mark[0] && rx_mark[1] && rx_mark[2] && rx_mark[3]) {
        memcpy((uint8_t *)&config_pid[0], rx_buf, 4);
      }
      if (rx_mark[4] && rx_mark[5] && rx_mark[6] && rx_mark[7]) {
        memcpy((uint8_t *)&config_pid[1], (uint8_t *)&rx_buf[4], 4);
      }
      if (rx_mark[8] && rx_mark[9] && rx_mark[10] && rx_mark[11]) {
        memcpy((uint8_t *)&config_pid[2], (uint8_t *)&rx_buf[8], 4);
      }  
      PIDInit(&pid_ctrl_pos_t, 
          (float)config_pid[0]/100.0f, 
          (float)config_pid[1]/100.0f, 
          (float)config_pid[2]/100.0f, 0.001, 
          lowest_pwm, 2047, AUTOMATIC, DIRECT);  
      pid_ctrl_pos_t.setpoint = setting_rpm * pole_pairs / 60;
    }
    else if (rx_data[0] == 0x60) {
      motor_dir = rx_data[1];   
      SetMotorDir(motor_dir);
    }
    else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x71) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x70+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x70+i] = 1;     
      } 
      if (rx_mark[0]) {
        motor_model = rx_buf[0];
        if (motor_model)
          HAL_GPIO_WritePin(DRV_FS_GPIO_Port, DRV_FS_Pin, GPIO_PIN_SET);
        else
          HAL_GPIO_WritePin(DRV_FS_GPIO_Port, DRV_FS_Pin, GPIO_PIN_RESET);
      }      
      if (rx_mark[1]) {
        pole_pairs = rx_buf[1];
        if (!pole_pairs)
          pole_pairs = 1;
      }          
    }
    else if (rx_data[0] >= 0xD0 && rx_data[0] <= 0xD3) {
      int32_t setting_rpm_int = 0;
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0xD0+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0xD0+i] = 1;     
      } 
      if (rx_mark[0]) {
        setting_rpm_int = rx_buf[0];
      }      
      if (rx_mark[1]) {
        setting_rpm_int &= 0xffff00ff;
        setting_rpm_int |= (rx_buf[1] << 8);
      }       
      if (rx_mark[2]) {
        setting_rpm_int &= 0xff00ffff;
        setting_rpm_int |= (rx_buf[2] << 16);
      }       
      if (rx_mark[3]) {
        setting_rpm_int &= 0x00ffffff;
        setting_rpm_int |= (rx_buf[3] << 24);
      }       
      setting_rpm = (float)setting_rpm_int / 100.0f;
      pid_ctrl_pos_t.setpoint = setting_rpm * pole_pairs / 60;
    }
    else if (rx_data[0] == 0xF0) {
      if (rx_data[1]) {
        flash_data_write_back();
      }         
    }
    else if (len > 1 && rx_data[0] == 0xFF) 
    {
      if (rx_data[1] < 128) {
        i2c_address[0] = rx_data[1];
        flash_data_write_back();
        user_i2c_init();
      }
    }
    else if (rx_data[0] >= 0xE0 && rx_data[0] <= 0xE1) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0xE0+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0xE0+i] = 1;     
      }  
      if (rx_mark[0]) {
        lowest_pwm = rx_buf[0];
      }      
      if (rx_mark[1]) {
        lowest_pwm &= 0x00ff;
        lowest_pwm |= (rx_buf[1] << 8);
      }   
      if (lowest_pwm > 2047)
        lowest_pwm = 2047;
      pid_ctrl_pos_t.outMin = lowest_pwm;         
    }
    else if (rx_data[0] == 0xFD)
    {
      if (rx_data[1] == 1) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_TIM_Base_DeInit(&htim3);
        HAL_TIM_Base_DeInit(&htim14);
        NVIC_SystemReset();        
      }
    }             
  }     
  else if (len == 1) {
    if (rx_data[0] == 0) {
      i2c1_set_send_data((uint8_t *)&motor_mode, 1);
    }
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x11) {
      i2c1_set_send_data((uint8_t *)&motor_pwm, 2);
    }
    else if (rx_data[0] >= 0x20 && rx_data[0] <= 0x23) {
      i2c1_set_send_data((uint8_t *)&rpm, 4);
    }
    else if (rx_data[0] >= 0x30 && rx_data[0] <= 0x33) {
      i2c1_set_send_data((uint8_t *)&speed, 4);
    }
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      i2c1_set_send_data((uint8_t *)&setting_rpm, 4);
    }
    else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x5B) {
      memcpy(tx_buf, (uint8_t *)&config_pid[0], 12);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x50], 0x5B-rx_data[0]+1);      
    }    
    else if (rx_data[0] == 0x60) {
      i2c1_set_send_data((uint8_t *)&motor_dir, 1);     
    }   
    else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x71) {
      tx_buf[0] = motor_model;
      tx_buf[1] = pole_pairs;
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x70], 0x71-rx_data[0]+1);      
    }  
    else if (rx_data[0] == 0x80) {
      i2c1_set_send_data((uint8_t *)&motor_status, 1);     
    }  
    else if (rx_data[0] >= 0x90 && rx_data[0] <= 0x93) {
      int32_t rpm_int = rpm * 100;
      i2c1_set_send_data((uint8_t *)&rpm_int, 4);
    }            
    else if (rx_data[0] >= 0xA0 && rx_data[0] <= 0xA3) {
      int32_t speed_int = speed * 100;
      i2c1_set_send_data((uint8_t *)&speed_int, 4);
    }          
    else if (rx_data[0] >= 0xB0 && rx_data[0] <= 0xBF) {
      char s[16]={"\0"};
      snprintf(s, 15, "%.2f", rpm);
      i2c1_set_send_data((uint8_t *)&s, 16);
    } 
    else if (rx_data[0] >= 0xC0 && rx_data[0] <= 0xCF) {
      char s[16]={"\0"};
      snprintf(s, 15, "%.2f", speed);      
      i2c1_set_send_data((uint8_t *)&s, 16);
    }  
    else if (rx_data[0] >= 0xD0 && rx_data[0] <= 0xD3) {
      int32_t setting_rpm_int = setting_rpm * 100;
      i2c1_set_send_data((uint8_t *)&setting_rpm_int, 4);
    }              
    else if (rx_data[0] >= 0xE0 && rx_data[0] <= 0xE1) {
      tx_buf[0] = lowest_pwm;
      tx_buf[1] = (lowest_pwm >> 8);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0xE0], 0xE1-rx_data[0]+1); 
    }
    else if (rx_data[0] == 0xFE) {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);     
    }                   
    else if (rx_data[0] == 0xFF) {
      i2c1_set_send_data((uint8_t *)&i2c_address[0], 1);     
    }                   
  }     
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  // set_pwm(1500);
  init_flash_data();
  user_i2c_init();
  if (motor_mode)
    init_pid();
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); 
  HAL_I2C_EnableListen_IT(&hi2c2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (trans_state) {
      if (i2c_timeout_delay < HAL_GetTick()) {
        i2c_timeout_delay = HAL_GetTick() + 505;
        trans_timeout_state += 1;
        if (trans_timeout_state >= 2) {
          reset_i2c_bus();
          HAL_Delay(50);
        }        
      }
    }    
    boot_success = is_boot_success();
    if(TIM3CH1_CAPTURE_STA&0X80)        //成功捕获到了�??次高电平
		{
			temp=TIM3CH1_CAPTURE_STA&0X3F; 
			temp*=65535;		 	    //溢出时间总和
			temp+=TIM3CH1_CAPTURE_VAL;      //得到总的高电平时�??
			//printf("HIGH:%lld us\r\n",temp);//打印总的高点平时�??, us
			TIM3CH1_CAPTURE_STA=0;          //�??启下�??次捕�??
      speed = 500000.0/temp;
      // if (avg_filter_level != 0) {
      //   speed_record[record_index] = speed;
      //   record_index++;
      //   if (record_index >= avg_filter_level) {
      //     record_index = 0;
      //   }
      //   speed = avg_filter(speed_record, avg_filter_level);
      // }      
      speed = lowPassFilter(speed);
      rpm = speed * 60 / pole_pairs;
		} 
    if (motor_mode) {
      if (!boot_success) {
        motor_boot_err_flag = 0;
        speed_pid();
        reboot_delay = HAL_GetTick();
      }
      else {
        speed = 0;
        rpm = 0;
        if (HAL_GetTick() - reboot_delay >= 2000) {
          if (pid_ctrl_pos_t.setpoint > 1) {
            set_pwm(0);
            HAL_Delay(550);
            uint16_t reboot_pwm = motor_boot_err_flag * 100 + 100;
            set_pwm(reboot_pwm > 2047 ? 2047 : reboot_pwm);
            boot_success = is_boot_success();
            reboot_delay = HAL_GetTick();
            while (boot_success)
            {
              if (motor_status == MOTOR_ERROR) {
                if (flash_led_delay < HAL_GetTick()) {
                  flash_led_delay = HAL_GetTick() + 100;
                  if (flash_led_flag) {
                    flash_led_flag = 0;
                    HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET);
                  }
                  else {
                    flash_led_flag = 1;
                    HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET);
                  }
                }
              }              
              boot_success = is_boot_success();
              if (!boot_success)  break;
              if (HAL_GetTick() - reboot_delay >= 2000) {
                motor_boot_err_flag++;
                break;
              }
            }
          }
          reboot_delay = HAL_GetTick();
        }
        if (motor_boot_err_flag >= 10) {
          motor_status = MOTOR_ERROR;
        }
      }
    }
    else {
      if (boot_success) {
        speed = 0;
        rpm = 0;        
      }
      if (!motor_pwm) {
        motor_status = MOTOR_STANBY;
      }
      else {
        motor_status = MOTOR_RUNNING;
      }
    }

    if (motor_status == MOTOR_RUNNING) {
      if (flash_led_delay < HAL_GetTick()) {
        flash_led_delay = HAL_GetTick() + 500;
        if (flash_led_flag) {
          flash_led_flag = 0;
          HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET);
        }
        else {
          flash_led_flag = 1;
          HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET);
        }
      }
    }
    else if (motor_status == MOTOR_ERROR) {
      if (flash_led_delay < HAL_GetTick()) {
        flash_led_delay = HAL_GetTick() + 100;
        if (flash_led_flag) {
          flash_led_flag = 0;
          HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET);
        }
        else {
          flash_led_flag = 1;
          HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET);
        }
      }
    }
    else if (motor_status == MOTOR_STANBY) {
      HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
