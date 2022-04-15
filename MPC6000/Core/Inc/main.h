/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h> // Includes for uint*_t types

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum IO { IN, OUT };

enum GPIO_MODE_E
    {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_ALT_FUNC,
    GPIO_ANALOG   // reset mode
    };


typedef struct GPIO
    {
    enum GPIO_MODE_E pin_mode;
    char        port;
    uint8_t  pin_num;
    } GPIO_t;

/* ----------------------------------------------------------------------------
 * struct Bio_Data
 *
 * Encapsulation of the state for a Bio_data Received.
 *
 * -------------------------------------------------------------------------- */

typedef struct Bio_Data
    {
    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate; // LSB = 0.1bpm
    uint8_t  confidence; // 0-100% LSB = 1%
    uint16_t oxygen; // 0-100% LSB = 1%
    uint8_t  status; // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
    float    rValue;      // -- Algorithm Mode 2 vv
    int8_t   extStatus;   // --
    uint8_t  reserveOne;  // --
    uint8_t  resserveTwo; // -- Algorithm Mode 2 ^^
    } Bio_Data_t;

  // The family register bytes are the larger umbrella for all the Index and
  // Write Bytes listed below. You can not reference a nestled byte without first
  // referencing it's larger category: Family Register Byte.
  enum FAMILY_REGISTER_BYTES {

    HUB_STATUS               = 0x00,
    SET_DEVICE_MODE,
    READ_DEVICE_MODE,
    OUTPUT_MODE            = 0x10,
    READ_OUTPUT_MODE,
    READ_DATA_OUTPUT,
    READ_DATA_INPUT,
    WRITE_INPUT,
    WRITE_REGISTER           = 0x40,
    READ_REGISTER,
    READ_ATTRIBUTES_AFE,
    DUMP_REGISTERS,
    ENABLE_SENSOR,
    READ_SENSOR_MODE,
    CHANGE_ALGORITHM_CONFIG  = 0x50,
    READ_ALGORITHM_CONFIG,
    ENABLE_ALGORITHM,
    BOOTLOADER_FLASH         = 0x80,
    BOOTLOADER_INFO,
    IDENTITY                 = 0xFF

  };

  // Status bytes for when I2C transmission and indicates what the status is for these.
  enum READ_STATUS_BYTE_VALUE
      {
      O2_SUCCESS                  = 0x00,
      ERR_UNAVAIL_CMD,
      ERR_UNAVAIL_FUNC,
      ERR_DATA_FORMAT,
      ERR_INPUT_VALUE,
      ERR_TRY_AGAIN,
      ERR_BTLDR_GENERAL        = 0x80,
      ERR_BTLDR_CHECKSUM,
      ERR_BTLDR_AUTH,
      ERR_BTLDR_INVALID_APP,
      ERR_UNKNOWN              = 0xFF
      };

/* ----------------------------------------------------------------------------
 * struct SparkFun_Bio_Sensor
 *
 * Encapsulation of the state for the sensor.
 *
 * -------------------------------------------------------------------------- */
typedef struct SparkFun_Bio_Sensor
    {
    // Internal State
    I2C_HandleTypeDef *_i2c_h; // i2c_handler
    GPIO_t _reset_pin;
    GPIO_t _mfio_pin;
    uint8_t _addr;
    uint8_t _userSelectedMode;
    uint8_t _sampleRate; // = 100;
    } SparkFun_Bio_Sensor_t;

struct Ssor_version {
  // 3 bytes total
  uint8_t major;
  uint8_t minor;
  uint8_t revision;
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define WRITE_FIFO_INPUT_BYTE  0x04
#define DISABLE                0x00
#define ENABLE                 0x01
#define MODE_ONE               0x01
#define MODE_TWO               0x02
#define APP_MODE               0x00
#define BOOTLOADER_MODE        0x08
#define NO_WRITE               0x00
#define INCORR_PARAM           0xEE

#define CONFIGURATION_REGISTER 0x0A
#define PULSE_MASK             0xFC
#define READ_PULSE_MASK        0x03
#define SAMP_MASK              0xE3
#define READ_SAMP_MASK         0x1C
#define ADC_MASK               0x9F
#define READ_ADC_MASK          0x60

#define ENABLE_CMD_DELAY          45 // Milliseconds
#define CMD_DELAY                 6  // Milliseconds
#define MAXFAST_ARRAY_SIZE        6  // Number of bytes....
#define MAXFAST_EXTENDED_DATA     5
#define MAX30101_LED_ARRAY        12 // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT             0x00
#define READ_FORMAT            0x01 // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD    0x01 //Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

#define DEFAULT_sAMPLE_RATE    100
extern const uint8_t bio_addr_c;
extern uint8_t const * const algo_arr_ptr;
extern const size_t algo_arr_size;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void config_gpio( const char port, const int pin_num, const enum IO direction );
void write_gpio( const char port, const int pin_num, const GPIO_PinState bit_of );
GPIO_PinState read_gpio( const char port, const int pin_num );

void set_pin_mode( struct GPIO * const gpio, const enum IO direction );
enum GPIO_MODE_E read_gpio_state( const char port, const int pin_num );
enum GPIO_MODE_E read_gpio_t_state( struct GPIO const * const gpio );

void write_gpio_t( struct GPIO * const gpio, const GPIO_PinState bit_of );
GPIO_PinState read_gpio_t( struct GPIO const * const gpio );

//------------------------------------------------------------------------------------------------
//
//                                 SparkFun_Bio_Sensor Member Function Declarations
//
//------------------------------------------------------------------------------------------------

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled HIGH while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in application mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x00 which is the byte indicating
// which mode the IC is in.
uint8_t enter_app_mode( struct SparkFun_Bio_Sensor * const bio_ssor );

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in bootloader mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x08 which is the byte indicating
// that the board is in bootloader mode.
uint8_t enter_bootloader( struct SparkFun_Bio_Sensor * const bio_ssor );

// This function initiates the bio_ssor structure (filling it with arguments passed)
// as well as putting the sensor into a particular mode (as specified)
void bio_sensor_init( struct SparkFun_Bio_Sensor * const bio_ssor, I2C_HandleTypeDef *const i2c_h, const uint8_t addr, const GPIO_t rst_pin, const GPIO_t mfio_pin, const uint8_t sample_rate, const uint8_t user_sel_mode );

uint8_t write_byte( struct SparkFun_Bio_Sensor * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, const uint8_t write_byte );
uint8_t write_2_bytes( struct SparkFun_Bio_Sensor * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, const uint8_t write_byte, const uint8_t write_byte_2 );

// This function is the same as the function above and uses the given family,
// index, and write byte, but also takes a 16 bit integer as a paramter to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t write_byte_and_16b_int( struct SparkFun_Bio_Sensor * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, const uint8_t write_byte, const uint16_t val_16_bit );
uint8_t write_bytes_arb( struct SparkFun_Bio_Sensor * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, const uint8_t byte_arr[ ], const uint32_t arr_size );

uint8_t read_byte( struct SparkFun_Bio_Sensor const * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte );

// This is similar to read_byte but also allows for a write byte to be passed as an argument.
// This is added just in case it's needed. It starts a request by writing the family, index,
// and write byte to the MAX32664 and reads the data that returns.
uint8_t read_byte_w_write_byte( struct SparkFun_Bio_Sensor const * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, const uint8_t write_byte );

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns multiple requested bytes.
uint8_t read_multiple_bytes( struct SparkFun_Bio_Sensor const * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, const uint8_t write_byte, uint8_t byte_arr[], const size_t arr_size );

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
