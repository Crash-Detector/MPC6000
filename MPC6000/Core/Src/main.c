/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include "HAL_FONA.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU_SampleRate 80 /* in Hz */
#define FALL_DETECT_SAMPLES 40 /*in samples .. = 0.5 * MPU_SampleRate */
#define Write_HM 0xAA
#define Read_HM 0xAB
#define HM_ADDR 0x55 // Without w/r bit.


//#define ACC_LFT_SQ    0.09 /* 0.3 g */
//#define ACC_UFT_SQ 7.67 /* 2.77 g */
//#define GYR_UFT_SQ 64719.36 /* in 254.4 deg/s */

/* for demo only */
#define ACC_LFT_SQ 1.44 /* for demo */
#define ACC_UFT_SQ 7.67 /* 2.77 g */
#define GYR_UFT_SQ 0 /* for demo */

int fall_detected = 0;
// int initialized = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
extern char const * const ok_reply_c;
extern const int reply_buff_size_c;
extern const int fona_def_timeout_ms_c;

extern const HAL_GPIO_t rst_pin; // PF13
extern const HAL_GPIO_t pwr_pin; // PE09

static const GPIO_t rst_pin_c  = { GPIO_OUTPUT, 'D', 0 };
static const GPIO_t mfio_pin_c = { GPIO_OUTPUT, 'D', 1 };
static const uint8_t def_sample_rate = 100; // 100 Hz

char message_buffer[1024];
char const * const init_mess ="Hello World!";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* MPU is connected to I2C3.
 * Here is the code that hanles I2C reads and writes to MPU
 * */

#define MPU_SAD_R 0b11010001 // The last bit corresponds to R
#define MPU_SAD_W 0b11010000

#define MPU_SAD 0b1101000
uint8_t MPUbuf[10] = {0};
void readMPU(uint8_t* val, uint8_t reg_addr, size_t len){
      HAL_StatusTypeDef ret;
      MPUbuf[0] = reg_addr;
      ret = HAL_I2C_Master_Transmit(&hi2c3, MPU_SAD_W, &MPUbuf[0], 1, 1000);
      if (ret != HAL_OK) {
          printf("Error reading Data from MPU reg: %d \n", reg_addr);
          return;
      }
      ret = HAL_I2C_Master_Receive(&hi2c3, MPU_SAD_R, &MPUbuf[0], len, 1000);
      if (ret != HAL_OK) printf("Error reading Data from MPU reg: %d size: %d\n", reg_addr, len);
      for(size_t i=0; i<len; i++)
          val[i] = MPUbuf[i];
}

void writeMPU(uint8_t val, uint8_t reg_addr){
    HAL_StatusTypeDef ret;
    MPUbuf[0] = reg_addr;
    MPUbuf[1] = val;
    ret = HAL_I2C_Master_Transmit(&hi2c3, MPU_SAD_W, &MPUbuf[0], 2, 1000);
    if (ret != HAL_OK) printf("Error writing to MPU reg: %d = %d\n", reg_addr, val);
}

#define MPU_SMPRT_DIV           0x19
#define MPU_CONFIG_REG          0x1a
#define MPU_GYRO_CONFIG         0x1b
#define MPU_ACC_CONFIG          0x1c
#define MPU_PWR_MGMT_1          0x6b
#define MPU_WHO_AM_I            0x75
#define MPU_INT_ENABLE          0x38
#define MPU_INT_STATUS          0x3a

/* measurements*/
#define MPU_ACC_X_OUT           0x3b
#define MPU_GYRO_X_OUT          0x43

#define MPU_OUTPUT_RATE         8000 // default output rate in Hz

void SetupMPU(){
      uint8_t mpu_id;
      readMPU(&mpu_id, MPU_WHO_AM_I, 1);
      printf("Setting up MPU Device on I2C3...\n");
      if (mpu_id != 104) {
          printf("[ERROR] MPU Device Setup Failed!!!\n");
          exit(1);
      }
      // reset and wait up from sleep
      uint8_t mpu_pwr_1= 0b10000000;
      writeMPU(mpu_pwr_1, MPU_PWR_MGMT_1);
      HAL_Delay(100);
      mpu_pwr_1 = 0;
      writeMPU(mpu_pwr_1, MPU_PWR_MGMT_1);

      // config sampling rate
      uint8_t mpu_sample_div = MPU_OUTPUT_RATE / MPU_SampleRate;
      writeMPU(mpu_sample_div, MPU_SMPRT_DIV);

      // config reg
      uint8_t mpu_config_reg;
      mpu_config_reg = 0b001 << 3;
      writeMPU(mpu_config_reg, MPU_CONFIG_REG);


      // gyro config
      uint8_t mpu_gyro_config = 0b11 << 3;
      writeMPU(mpu_gyro_config, MPU_GYRO_CONFIG);
      readMPU(&mpu_gyro_config, MPU_GYRO_CONFIG, 1);
      if (mpu_gyro_config != 0b11 << 3) {
          printf("[ERROR] MPU GyroMeter Setup Failed!!!");
          exit(1);
      }

      // Acc config
      uint8_t mpu_acc_config;
      mpu_acc_config = 0b11 << 3;
      writeMPU(mpu_acc_config, MPU_ACC_CONFIG);
      readMPU(&mpu_acc_config, MPU_ACC_CONFIG, 1);
      if (mpu_acc_config != 0b11 << 3) {
          printf("[ERROR] MPU Acc Setup Failed!!!\n");
          exit(1);
      }

      // Generate interrupt at each data ready
      // set DATA_RDY_EN = 1
      uint8_t mpu_int_enable;
      mpu_int_enable = 0b1;
      writeMPU(mpu_int_enable, MPU_INT_ENABLE);
      readMPU(&mpu_int_enable, MPU_INT_ENABLE, 1);
      if (mpu_int_enable != 1) {
          printf("[ERROR] MPU interrupt Setup Failed!!!\n");
          exit(1);
      }

      printf("...MPU Setup Success\n");
}

void MPUSleep(){
    uint8_t mpu_pwr_1 = 1 << 6;
      writeMPU(mpu_pwr_1, MPU_PWR_MGMT_1);
}

typedef struct MPU_measure{
    float Accx;
    float Accy;
    float Accz;
    float Gyrx;
    float Gyry;
    float Gyrz;
}MPU_measure;

MPU_measure getMPU(){
    MPU_measure rt;
    // Read from x-axis:
      uint8_t raw_acc[6];
      readMPU(raw_acc, MPU_ACC_X_OUT, 6);
      int16_t raw_x, raw_y, raw_z;
      raw_x = raw_acc[0] << 8 | raw_acc[1];
      raw_y = raw_acc[2] << 8 | raw_acc[3];
      raw_z = raw_acc[4] << 8 | raw_acc[5];
      rt.Accx = (float)(raw_x)/2048.0;
      rt.Accy = (float)(raw_y)/2048.0;
      rt.Accz = (float)(raw_z)/2048.0;
//    printf("Acc X: %f Gs Y: %f Gs Z: %f Gs \n", Accx, Accy, Accz);
      uint8_t raw_gyro[6];
      readMPU(raw_gyro, MPU_GYRO_X_OUT, 6);
      int16_t raw_x_g, raw_y_g, raw_z_g;
      raw_x_g = raw_gyro[0] << 8 | raw_gyro[1];
      raw_y_g = raw_gyro[2] << 8 | raw_gyro[3];
      raw_z_g = raw_gyro[4] << 8 | raw_gyro[5];
      rt.Gyrx = (float)(raw_x_g)/65.532;
      rt.Gyry = (float)(raw_y_g)/65.532;
      rt.Gyrz = (float)(raw_z_g)/65.532;
//    printf("Gyro X: %f deg/s Y: %f deg/s Z: %f deg/s\n", Gx, Gy, Gz);
      return rt;
}

unsigned int fall_window=0;


/* pushes MPU data to usb */

void detect_fall(MPU_measure m){
    float accsq;
    accsq = m.Accx * m.Accx + m.Accy * m.Accy + m.Accz * m.Accz;
    if (accsq <= ACC_LFT_SQ && !fall_window)
        fall_window = FALL_DETECT_SAMPLES;
    if (!fall_window) return;
    fall_window--;
    if (accsq < ACC_UFT_SQ) return;
    float gyrsq;
    gyrsq = m.Gyrx * m.Gyrx + m.Gyry * m.Gyry + m.Gyrz * m.Gyrz;
    if (gyrsq < GYR_UFT_SQ) return;
    fall_detected = 1;
}
void push_MPU_data(MPU_measure m){
    printf("%f,%f,%f,%f,%f,%f,%d,%d\n", m.Accx, m.Accy, m.Accz, m.Gyrx, m.Gyry, m.Gyrz, fall_window, fall_detected);
}

void MPU_Interrupt(){
    /* read MPU */
    MPU_measure mpu_data = getMPU();
    push_MPU_data(mpu_data);
    detect_fall(mpu_data);

    /* clear interrupt*/
    uint8_t mpu_int_status;
    readMPU(&mpu_int_status, MPU_INT_STATUS, 1);
    if ((mpu_int_status & 0b1) != 1) {
        printf("[ERROR] MPU interrupt Clear Failed!!!!\n");
        exit(1);
    }
}


void Beep_sos(){
    TIM4->ARR = 2000;
    for(int j=0; j<5; j++){
    for (int i=0; i<3; i++){
    TIM4->CCR2 = 1000;
    HAL_Delay(90);
    TIM4->CCR2 = 0;
    HAL_Delay(90);
    }
    for (int i=0; i<3; i++){
        TIM4->CCR2 = 1000;
        HAL_Delay(180);
        TIM4->CCR2 = 0;
        HAL_Delay(90);
        }
    for (int i=0; i<3; i++){
        TIM4->CCR2 = 1000;
        HAL_Delay(90);
        TIM4->CCR2 = 0;
        HAL_Delay(90);
    }
    HAL_Delay(300);
    }

}

void Beep_sendmsg_warning(){
    for (int i=0; i<6; i++){
        TIM4->ARR = 3000;
        TIM4->CCR2 = 1500;
        HAL_Delay(150);
        TIM4->ARR = 2000;
        TIM4->CCR2 = 1000;
        HAL_Delay(150);
    }
    TIM4->CCR2 =0;

}

void Beep_reset(){
    TIM4->ARR = 2000;
    TIM4->CCR2 = 1000;
    HAL_Delay(20);
    TIM4->ARR = 4000;
    TIM4->CCR2 = 2000;
    HAL_Delay(20);
    TIM4->CCR2 =0;
    printf( "Reset beeper!\n\r" );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  Cellular_module_t cell;
  SparkFun_Bio_Sensor_t sensor;

  NVIC_DisableIRQ(EXTI4_IRQn);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  cell.uart_ptr = &huart3;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n");

  printf( "Initializing!\n\r" );
  if ( !begin( &cell ) )
    {
    printf( "Failed initialization\n\r" );
    return 1;
    }
  else
    {
    printf( "Found SIM7000 using hardware serial\n\r" );
    }

  if ( !setNetworkSettings( &cell ) )
    {
    printf( "Network settings NOT set\n\r" );
    return 1;
    }
  else
    {
    printf( "Network settings set\n\r" );
    }
  // initialized = 1;


  /*biometric sensor setup*/
  /*--------------begin------------------*/
	  HAL_StatusTypeDef ret;
	  uint8_t buf[10];
	  int HM_samples = 0x1;
  	  bio_sensor_init( &sensor, &hi2c1, HM_ADDR, rst_pin_c, mfio_pin_c, def_sample_rate, DISABLE );
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  0); // Reset pin
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  1); // MFIO pin
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  1); // Reset is pulled
	  HAL_Delay(50);
	  HAL_Delay(1000);

	  buf[0] = 0x02;
	  buf[1] = 0x00;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
	  HAL_Delay(6);
	  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 2, 5000);
	  printf("error code: %x application mode: %x\n\r", buf[0],buf[1]);
	  set_pin_mode( &sensor._mfio_pin, IN );

	  /*set our mode to MODE 1*/
	    buf[0] = 0x10;
	    buf[1] = 0x00;
	    buf[2] = 0x02;
	    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
	    HAL_Delay(6);
	    ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
	    if(buf[0] != 0x00 || ret != HAL_OK ){
	        printf("Error setting mode: code %x\n\r", buf[0]);
	    }
	    printf("mode set to raw and algo\n");

	    /*Set FIFO threshold as almost full at 0x0F*/
	    buf[0] = 0x10;
	    buf[1] = 0x01;
	    buf[2] = 0x1;
	    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
	    HAL_Delay(6);
	    ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
	      if(buf[0] != 0x00 || ret != HAL_OK ){
	        printf("Error setting FIFO threshold code: %x\n\r", buf[0]);
	      }
	    printf("fifo set\n");
	    /*disable AGC*/
	    buf[0] = 0x52;
	      buf[1] = 0x00;
	      buf[2] = 0x1;
	      ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
	      HAL_Delay(25);
	      ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
	        if(buf[0] != 0x00 || ret != HAL_OK ){
	          printf("Error Enabling Algorithm code: %x\n\r", buf[0]);
	        }
	       printf("enable AGC \n");



	    /*Disable the sensor*/
	    buf[0] = 0x44;
	    buf[1] = 0x03;
	    buf[2] = 0x1;
	    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
	    HAL_Delay(45);
	    ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
	      if(buf[0] != 0x00 || ret != HAL_OK ){
	        printf("Error enabling sensor code: %x\n\r", buf[0]);
	      }
	      printf("sensor set\n");


	    /*Disable the algorithm*/

	      HAL_Delay(500);
	    buf[0] = 0x52;
	    buf[1] = 0x02;
	    buf[2] = 0x1;
	    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
	    HAL_Delay(45);
	    ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);

	      if(buf[0] != 0x00 || ret != HAL_OK ){
	        printf("Error Enabling Algorithm code: %x\n\r", buf[0]);
	      }
	      printf("enable algorithm \n");




	    HAL_Delay(1000);

  /*--------------end------------------*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  fall_detected = 0;
  Beep_reset(); // Ok to enable interrupts now
  SetupMPU();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  TIM4->CCR2 = 0;

  NVIC_EnableIRQ(EXTI4_IRQn);
    while(1)
        {
        if (fall_detected)
            {
            MPUSleep(); /* turn off MPU */
            Beep_sos(); /* 30 sec */
            Beep_sendmsg_warning(); /* 3 sec */



            /*Enable the sensor*/
            					/*enable AGC*/
								buf[0] = 0x52;
								buf[1] = 0x00;
								buf[2] = 0x01;
								ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
								HAL_Delay(25);
								ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
								if(buf[0] != 0x00 || ret != HAL_OK ){
									printf("Error Enabling Algorithm code: %x\n\r", buf[0]);
								}
								printf("enable AGC \n");



//								/*Enable the sensor*/
//								buf[0] = 0x44;
//								buf[1] = 0x03;
//								buf[2] = 0x00;
//								ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
//								HAL_Delay(45);
//								ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
//								  if(buf[0] != 0x00 || ret != HAL_OK ){
//									printf("Error enabling sensor code: %x\n\r", buf[0]);
//								  }
//								  printf("sensor set\n");
//
//
//
//            					buf[0] = 0x44;
//            					buf[1] = 0x03;
//            					buf[2] = 0x01;
//            					ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
//            					HAL_Delay(45);
//            					ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
//            					if(buf[0] != 0x00 || ret != HAL_OK ){
//            						printf("Error enabling sensor code: %x\n\r", buf[0]);
//            					}
//            						printf("sensor set\n");
//
//
//            						/*Enable the algorithm*/
//
//            					HAL_Delay(500);
//            					buf[0] = 0x52;
//            					buf[1] = 0x02;
//            					buf[2] = 0x01;
//            					ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
//            					HAL_Delay(45);
//            					ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
//
//            					if(buf[0] != 0x00 || ret != HAL_OK ){
//            						printf("Error Enabling Algorithm code: %x\n\r", buf[0]);
//            					}
//            						printf("enable algorithm \n");
//
//            					HAL_Delay(1000); //wait for the data to load




            // read biometric sensor
            buf[0] = 0x12;
            buf[1] = 0x01;
            int heart_rate = 0;
            float blood_oxygen = 0;
            int status = 0;
            for(int i = 0; i < 40; ++i){
            	ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
				HAL_Delay(6);
				ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 10, 5000);
				printf("code: %d ", buf[0]);
				int heartRate = ((buf[1]) << 8);
				printf("%d %d %d %d\n", buf[1], buf[2], buf[3], buf[6]);
				heartRate |= (buf[2]);
				heartRate = heartRate/10;
				heart_rate = heartRate;
				// Confidence formatting
				int confidence = buf[3];

				//Blood oxygen level formatting
				int oxygen = (buf[4]) << 8;
				oxygen += buf[5];
				blood_oxygen = oxygen/10.0;
				// blood_oxygen = oxygen;

				//"Machine State" - has a finger been detected?

				status = buf[6];
				printf( "heartrate: %d, status: %d, blood_oxygen %f, confidence: %d\n\r", heartRate, status, oxygen, confidence );
				/*
				if(status == 3){
					if(heartRate != 0){
						heart_rate = heartRate;

					}
					if(oxygen != 0){
						blood_oxygen = oxygen;

					}

				}
				*/
				HAL_Delay(100);




            }

            //get GPS
            float latitude = 0;
            float longitude= 0;



            if(status != 3){
            	sprintf(message_buffer, "SOS! An athlete has fallen and been injured at %d latitude %d longitude. There finger is off the vitals sensor but we measured: heart rate: %d blood oxygen: %d", latitude, longitude, heart_rate, blood_oxygen);
            }
            else{
            	sprintf(message_buffer, "SOS! An athlete has fallen and been injured at %d latitude %d longitude. The vitals sensor measured: heart rate: %d blood oxygen: %d", latitude, longitude, heart_rate, blood_oxygen);
            }


            // send out message
            printf( "Here's the sos message: %s\n\r", message_buffer );
            HAL_Delay( 1000 );
            //sendSMS( &cell, message_buffer );
            fall_detected = 0;
            SetupMPU(); /* reset MPU */
            Beep_reset();
            } // end if
        } // end while
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00000E14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE10 PE11
                           PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

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

