
#ifndef HAL_FONA_H
#define HAL_FONA_H

// enum boolean { false, true };
#include <stdbool.h>   // For bool
#include <assert.h>
#include <stdint.h> // Allow for larger ints?
#include "main.h"

extern char const * const ok_reply_c;
extern const int reply_buff_size_c;
extern const int fona_def_timeout_ms_c;

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BUFF_SIZE 32
#define DEBUG_CELL
//#define REPLY_BUFF_SIZE 256
//#define FONA_DEFAULT_TIMEOUT_MS 500



/* USER CODE END EM */

typedef struct {
    UART_HandleTypeDef *uart_ptr;
    char reply_buffer[ 256 ];       // Place where reply lives (avoids malloc which has unpredictable behavior)
} Cellular_module_t;

typedef struct {
    GPIO_TypeDef* GPIOx     ;
    uint16_t      GPIO_Pin  ;
} HAL_GPIO_t;

void GPIO_Write( HAL_GPIO_t const * const gpio_ptr, const GPIO_PinState pin_state );

extern const HAL_GPIO_t rst_pin;
extern const HAL_GPIO_t pwr_pin;


//------------------------------------------------------------------------------------------------
//
//                                  UARTS Functions Declaration
//
//------------------------------------------------------------------------------------------------
// char read( UART_HandleTypeDef * const uart_ptr );

// Flushes all characters inside buffer.
void flushInput( UART_HandleTypeDef * const uart_ptr );
void println( UART_HandleTypeDef * const uart_ptr, const char * const message  );
uint8_t transmit( Cellular_module_t * const cell_ptr, char const * const send, uint16_t timeout );
typedef struct {
    size_t s_elem, n_elem;
    uint8_t buf[BUFF_SIZE];
    volatile size_t head;
    volatile size_t tail;
} Ring_Buffer_t;




bool begin( Cellular_module_t * const cell_ptr );
uint8_t readline( Cellular_module_t * const cell_ptr, const uint16_t timeout, const bool multiline );
bool send_check_reply( Cellular_module_t * const cell_ptr, char const * const send,
                        char const * const reply, const uint16_t timeout );
// int available( Cellular_module_t * const cell_mod_ptr );
/*
size_t write(Cellular_module_t * const cell_mod_ptr, uint8_t x);
bool read( Cellular_module_t * const cell_mod_ptr );
int peek(void);
void flush();
void flushInput( UART_HandleTypeDef * const uart_ptr ); // Flush input.

bool parseReply(  Cellular_module_t * const cell_mod_ptr, FONAFlashStringPtr toreply,
        uint16_t *v, char divider  = ',', uint8_t index=0);
bool parseReplyFloat( Cellular_module_t * const cell_mod_ptr, FONAFlashStringPtr toreply,
           float *f, char divider, uint8_t index);
bool parseReply(FONAFlashStringPtr toreply,
           char *v, char divider  = ',', uint8_t index=0);
bool parseReplyQuoted(FONAFlashStringPtr toreply,
           char *v, int maxlen, char divider, uint8_t index);

// Network connection (AT+CNACT)
bool openWirelessConnection(bool onoff);
bool wirelessConnStatus(void);

// GPS handling
bool enableGPS(bool onoff);
int8_t GPSstatus(void);
uint8_t getGPS(uint8_t arg, char *buffer, uint8_t maxbuff);
bool getGPS(float *lat, float *lon, float *speed_kph=0, float *heading=0, float *altitude=0);
//bool getGPS(float *lat, float *lon, float *speed_kph, float *heading, float *altitude,
// uint16_t *year = NULL, uint8_t *month = NULL, uint8_t *day = NULL, uint8_t *hour = NULL, uint8_t *min = NULL, float *sec = NULL);
bool enableGPSNMEA(uint8_t nmea);

// Power, battery, and ADC
void powerOn(uint8_t FONA_PWRKEY);
bool powerDown(void);
bool getADCVoltage(uint16_t *v);
bool getBattPercent(uint16_t *p);
bool getBattVoltage(uint16_t *v);

// Functionality and operation mode settings
bool setFunctionality(uint8_t option); // AT+CFUN command
bool enableSleepMode(bool onoff); // AT+CSCLK command

// SIM query
uint8_t unlockSIM(char *pin);
uint8_t getSIMCCID(char *ccid);
uint8_t getNetworkStatus(void);

// SMS handling
bool setSMSInterrupt(uint8_t i); // Probably can't be used since we can't see.
uint8_t getSMSInterrupt(void);      
int8_t getNumSMS(void);
bool readSMS(uint8_t i, char *smsbuff, uint16_t max, uint16_t *readsize);
bool sendSMS(const char *smsaddr, const char *smsmsg); //! Needed
bool deleteSMS(uint8_t i);
bool deleteAllSMS(void);
bool getSMSSender(uint8_t i, char *sender, int senderlen);
bool sendUSSD(char *ussdmsg, char *ussdbuff, uint16_t maxlen, uint16_t *readlen);
*/
#endif // HAL_FONA_H
