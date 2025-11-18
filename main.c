#include <stdio.h>
#include <stdint.h>
#include "diag/Trace.h"
#include "stm32f051x8.h"
#include "cmsis/cmsis_device.h"

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Definitions of registers and their bits are
   given in system/include/cmsis/stm32f051x8.h */


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

unsigned int Freq = 0;  // measured frequency value (global variable)
unsigned int Res = 0;   // measured resistance value (global variable)

#define POTENTIOMETER_ADC_CHANNEL   (ADC_CHSELR_CHSEL1)
#define POTENTIOMETER_GPIO_ANALOG   (GPIO_MODER_MODER1)
#define POTENTIOMETER_GPIO_NOPULL   (GPIO_PUPDR_PUPDR1)

#define FUNCTION_GENERATOR_EXTI_MASK (EXTI_IMR_MR2)
#define FUNCTION_GENERATOR_EXTI_FLAG (EXTI_PR_PR2)
#define TIMER_555_EXTI_MASK          (EXTI_IMR_MR3)
#define TIMER_555_EXTI_FLAG          (EXTI_PR_PR3)

// init functions
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);

void myADC_Init(void);
void myDAC_Init(void);

uint32_t Potentiometer_resistance();
uint32_t Potentiometer_voltage();
void read_DAC();

void oled_Write(unsigned char);
void oled_Write_Cmd(unsigned char);
void oled_Write_Data(unsigned char);
void oled_config(void);

void refresh_OLED(void);

SPI_HandleTypeDef SPI_Handle;

typedef enum
{
SIGNAL_SOURCE_555 = 0,
SIGNAL_SOURCE_FUNCTION_GENERATOR = 1
} signal_source_t;

static void handle_signal_capture(signal_source_t source);
static void process_measurement(void);
static const char *signal_name(signal_source_t source);

volatile uint8_t timerTriggered = 0;
volatile uint32_t inSig = 0;
static volatile uint8_t measurementReady = 0;
static volatile uint32_t capturedTimerTicks = 0;
static volatile signal_source_t lastCapturedSource = SIGNAL_SOURCE_555;


/*** Call this function to boost the STM32F0xx clock to 48 MHz ***/

void SystemClock48MHz( void )
{

// Disable the PLL
    RCC->CR &= ~(RCC_CR_PLLON);

// Wait for the PLL to unlock
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );

// Configure the PLL for 48-MHz system clock
	RCC->CFGR = 0x00280000;

// Enable the PLL
	RCC->CR |= RCC_CR_PLLON;

// Wait for the PLL to lock
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

// Switch the processor to the PLL clock source
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;
	
// Update the system with the new clock frequency
    SystemCoreClockUpdate();

}

/*****************************************************************/
// LED Display initialization commands
unsigned char oled_init_cmds[] =
{
    0xAE,
    0x20, 0x00,
    0x40,
    0xA0 | 0x01,
    0xA8, 0x40 - 1,
    0xC0 | 0x08,
    0xD3, 0x00,
    0xDA, 0x32,
    0xD5, 0x80,
    0xD9, 0x22,
    0xDB, 0x30,
    0x81, 0xFF,
    0xA4,
    0xA6,
    0xAD, 0x30,
    0x8D, 0x10,
    0xAE | 0x01,
    0xC0,
    0xA0
};

unsigned char Characters[][8] = {
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b01011111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // !
    {0b00000000, 0b00000111, 0b00000000, 0b00000111, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // "
    {0b00010100, 0b01111111, 0b00010100, 0b01111111, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // #
    {0b00100100, 0b00101010, 0b01111111, 0b00101010, 0b00010010,0b00000000, 0b00000000, 0b00000000},  // $
    {0b00100011, 0b00010011, 0b00001000, 0b01100100, 0b01100010,0b00000000, 0b00000000, 0b00000000},  // %
    {0b00110110, 0b01001001, 0b01010101, 0b00100010, 0b01010000,0b00000000, 0b00000000, 0b00000000},  // &
    {0b00000000, 0b00000101, 0b00000011, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // '
    {0b00000000, 0b00011100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // (
    {0b00000000, 0b01000001, 0b00100010, 0b00011100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // )
    {0b00010100, 0b00001000, 0b00111110, 0b00001000, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // *
    {0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // +
    {0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ,
    {0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // -
    {0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // .
    {0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // /
    {0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // 0
    {0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // 1
    {0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // 2
    {0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // 3
    {0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000,0b00000000, 0b00000000, 0b00000000},  // 4
    {0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001,0b00000000, 0b00000000, 0b00000000},  // 5
    {0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000,0b00000000, 0b00000000, 0b00000000},  // 6
    {0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // 7
    {0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // 8
    {0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110,0b00000000, 0b00000000, 0b00000000},  // 9
    {0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // :
    {0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ;
    {0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // <
    {0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // =
    {0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // >
    {0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // ?
    {0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // @
    {0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110,0b00000000, 0b00000000, 0b00000000},  // A
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // B
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010,0b00000000, 0b00000000, 0b00000000},  // C
    {0b01111111, 0b01000001, 0b01000001, 0b00100010, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // D
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // E
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // F
    {0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010,0b00000000, 0b00000000, 0b00000000},  // G
    {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // H
    {0b01000000, 0b01000001, 0b01111111, 0b01000001, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // I
    {0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // J
    {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // K
    {0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // L
    {0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // M
    {0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // N
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // O
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // P
    {0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110,0b00000000, 0b00000000, 0b00000000},  // Q
    {0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // R
    {0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // S
    {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // T
    {0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // U
    {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111,0b00000000, 0b00000000, 0b00000000},  // V
    {0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // W
    {0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011,0b00000000, 0b00000000, 0b00000000},  // X
    {0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // Y
    {0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011,0b00000000, 0b00000000, 0b00000000},  // Z
    {0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // [
    {0b00010101, 0b00010110, 0b01111100, 0b00010110, 0b00010101,0b00000000, 0b00000000, 0b00000000},  // back slash
    {0b00000000, 0b00000000, 0b00000000, 0b01000001, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // ]
    {0b00000100, 0b00000010, 0b00000001, 0b00000010, 0b00000100,0b00000000, 0b00000000, 0b00000000},  // ^
    {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // _
    {0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // `
    {0b00100000, 0b01010100, 0b01010100, 0b01010100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // a
    {0b01111111, 0b01001000, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // b
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // c
    {0b00111000, 0b01000100, 0b01000100, 0b01001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // d
    {0b00111000, 0b01010100, 0b01010100, 0b01010100, 0b00011000,0b00000000, 0b00000000, 0b00000000},  // e
    {0b00001000, 0b01111110, 0b00001001, 0b00000001, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // f
    {0b00001100, 0b01010010, 0b01010010, 0b01010010, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // g
    {0b01111111, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // h
    {0b00000000, 0b01000100, 0b01111101, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // i
    {0b00100000, 0b01000000, 0b01000100, 0b00111101, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // j
    {0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // k
    {0b00000000, 0b01000001, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // l
    {0b01111100, 0b00000100, 0b00011000, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // m
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // n
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // o
    {0b01111100, 0b00010100, 0b00010100, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // p
    {0b00001000, 0b00010100, 0b00010100, 0b00011000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // q
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // r
    {0b01001000, 0b01010100, 0b01010100, 0b01010100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // s
    {0b00000100, 0b00111111, 0b01000100, 0b01000000, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // t
    {0b00111100, 0b01000000, 0b01000000, 0b00100000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // u
    {0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // v
    {0b00111100, 0b01000000, 0b00111000, 0b01000000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // w
    {0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // x
    {0b00001100, 0b01010000, 0b01010000, 0b01010000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // y
    {0b01000100, 0b01100100, 0b01010100, 0b01001100, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // z
    {0b00000000, 0b00001000, 0b00110110, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // {
    {0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // |
    {0b00000000, 0b01000001, 0b00110110, 0b00001000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // }
    {0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // ~
    {0b00001000, 0b00011100, 0b00101010, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000}   // <-
};



int main(int argc, char* argv[])
{

	SystemClock48MHz();

	trace_printf("This is the Project\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

myGPIOA_Init(); // initialize I/O port PA
myGPIOB_Init(); // initialize PB
myGPIOC_Init(); // initialize PC
	myDAC_Init(); // initialize DAC
	myTIM2_Init(); //Initialize timer TIM2
	myADC_Init(); // initialize ADC
	oled_config(); // configure the screen
	refresh_OLED(); // clock the first tick of the screen so that it is up before any other interupts
	myEXTI_Init(); //initialize EXTI


	while (1)
	{
		read_DAC(); // read dac (cont. updates the dac value as well)
		process_measurement();

		refresh_OLED(); // refresh screen
	}
	return 0;
}



void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

        /* Configure PA0 as input for the user push button */
        GPIOA->MODER &= ~(GPIO_MODER_MODER0);
        GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

        /* Configure PA1 as analog input for the potentiometer */
        GPIOA->MODER &= ~(POTENTIOMETER_GPIO_ANALOG);
        GPIOA->MODER |= POTENTIOMETER_GPIO_ANALOG;
        GPIOA->PUPDR &= ~(POTENTIOMETER_GPIO_NOPULL);

        /* Configure PA4 as analog output for the DAC */
        GPIOA->MODER &= ~(GPIO_MODER_MODER4);
        GPIOA->MODER |= GPIO_MODER_MODER4;
        GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
	/* Start counting timer pulses */ //maybe not needed (remove comment later)
	TIM2->CR1 |= TIM_CR1_CEN;
}


void myEXTI_Init()
{
        /* Enable SYSCFG clock before configuring EXTI routing */
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

        /* Map EXTI lines to the proper ports */
        SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI2 | SYSCFG_EXTICR1_EXTI3);
        SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PB | SYSCFG_EXTICR1_EXTI3_PB);

        /* Configure rising-edge trigger for the button and signal inputs */
        EXTI->RTSR |= (EXTI_RTSR_TR0 | EXTI_RTSR_TR2 | EXTI_RTSR_TR3);

        /* Unmask EXTI lines: start with button and 555 timer enabled */
        EXTI->IMR |= (EXTI_IMR_MR0 | TIMER_555_EXTI_MASK);
        EXTI->IMR &= ~(FUNCTION_GENERATOR_EXTI_MASK);

        /* Clear any pending EXTI flags */
        EXTI->PR |= (EXTI_PR_PR0 | FUNCTION_GENERATOR_EXTI_FLAG | TIMER_555_EXTI_FLAG);

        /* Assign EXTI2_3 interrupt priority = 1 in NVIC */
        NVIC_SetPriority(EXTI2_3_IRQn,1);

        /* Enable EXTI2_3 interrupts in NVIC */
        NVIC_EnableIRQ(EXTI2_3_IRQn);

        /* Assign EXTI0_1 interrupt priority = 0 in NVIC */
        NVIC_SetPriority(EXTI0_1_IRQn,0);

        /* Enable EXTI0_1 interrupts in NVIC */
        NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f051x8.c */

void EXTI0_1_IRQHandler()
{
	if ((EXTI->PR & EXTI_PR_PR0) != 0) {
		while ((GPIOA->IDR & GPIO_IDR_0) != 0){}

		TIM2->CR1 &= ~(TIM_CR1_CEN);
		TIM2->CNT = 0;
		timerTriggered = 0;

		if (inSig == 0) {
			inSig = 1;
			EXTI->IMR &= ~(TIMER_555_EXTI_MASK);
			EXTI->IMR |= FUNCTION_GENERATOR_EXTI_MASK;
		} else {
			inSig = 0;
			EXTI->IMR &= ~(FUNCTION_GENERATOR_EXTI_MASK);
			EXTI->IMR |= TIMER_555_EXTI_MASK;
		}

		EXTI->PR |= EXTI_PR_PR0;
	}
}

static void handle_signal_capture(signal_source_t source)
{
	if (timerTriggered == 0U) {
		timerTriggered = 1U;
		TIM2->CNT = 0U;
		TIM2->CR1 |= TIM_CR1_CEN;
	} else {
		timerTriggered = 0U;
		TIM2->CR1 &= ~(TIM_CR1_CEN);
		capturedTimerTicks = TIM2->CNT;
		lastCapturedSource = source;
		measurementReady = 1U;
	}
}

static const char *signal_name(signal_source_t source)
{
	return (source == SIGNAL_SOURCE_FUNCTION_GENERATOR) ? "Function Generator" : "555";
}

void EXTI2_3_IRQHandler()
{
	if ((EXTI->PR & TIMER_555_EXTI_FLAG) != 0) {
		handle_signal_capture(SIGNAL_SOURCE_555);
		EXTI->PR |= TIMER_555_EXTI_FLAG;
	}

	if ((EXTI->PR & FUNCTION_GENERATOR_EXTI_FLAG) != 0) {
		handle_signal_capture(SIGNAL_SOURCE_FUNCTION_GENERATOR);
		EXTI->PR |= FUNCTION_GENERATOR_EXTI_FLAG;
	}
}

static void process_measurement(void)
{
	if (measurementReady == 0U) {
		return;
	}

	uint32_t localTicks;
	signal_source_t localSource;

	__disable_irq();
	localTicks = capturedTimerTicks;
	localSource = lastCapturedSource;
	measurementReady = 0U;
	__enable_irq();

	double freq = (localTicks != 0U) ? ((double)SystemCoreClock) / ((double)localTicks) : 0.0;
	Freq = (unsigned int)freq;
	Res = (unsigned int)Potentiometer_resistance();

	trace_printf("Source: %s\n", signal_name(localSource));
	trace_printf("The Frequency is: %u Hz\n", Freq);
	trace_printf("The Resistance is: %u ohms\n\n", Res);
}

// initialize ADC
void myADC_Init(void){

	// initalize clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

//	1. Ensure that ADEN=0
//	2. Set ADCAL=1
//	3. Wait until ADCAL=0
//	4. The calibration factor can be read from bits 6:0 of ADC_DR.

	if ((ADC1->CR & ADC_CR_ADEN) != 0){	// if aden is not 0
		ADC1->CR |= ADC_CR_ADDIS; // clear aden by addis (adc is disabled)
		while ((ADC1->CR & ADC_CR_ADEN) != 0); // timeout for adc disable
	}
	ADC1->CR |= ADC_CR_ADCAL; // calibrate by setting adcal
	while(ADC1->CR & ADC_CR_ADCAL); // timeout until calibration complete


        ADC1->CHSELR = POTENTIOMETER_ADC_CHANNEL; // select channel 1 (PA1) for ADC input

	ADC1->SMPR |= 7; // max smp = 0b111 = 7 (239.5 ADC clock cycles)


	// change configuration registers
	ADC1->CFGR1 &= ~ADC_CFGR1_RES; // enable data resolution

	ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; // right align converted data

	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; // enable overrun management mode

	ADC1->CFGR1 |= ADC_CFGR1_CONT; // enable continuous conversion mode


	ADC1->CR |= ADC_CR_ADEN; // re enable ADC now that configuration register are changed

	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0); // timeout until ADC re enabled

	ADC1->CR |= ADC_CR_ADSTART; // ADC start (works as aden has been re enabkled)

	trace_printf("ADC initialized\n");
}

// initialize DAC
void myDAC_Init(void){

	RCC->APB1ENR |= RCC_APB1ENR_DACEN;  // enable dac clock

	DAC->CR |= DAC_CR_EN1; // enable channel1 of DAC

	trace_printf("DAC initialized\n");
}

uint32_t Potentiometer_resistance(){
	uint32_t ADC_value = Potentiometer_voltage();

	// Vchannel = ADC_data x (Vdda / Full_scale)
	// ADC_data = ADC_value (read)
	// Vdda = 3.3 V (given)
	// Full_scale = 2^12 - 1 = 4095 with 12 bit resolution (maximum digital value of the ADC output)

	float v_channel = (ADC_value * 3.3f) / (4095.0f);// 4095 = 2^12 - 1
	
	return 5000.0f * (v_channel / (3.3f - v_channel)); // resistance in circuit is given as 5k ohms
}

uint32_t Potentiometer_voltage(){
	return (ADC1->DR & 0xFFF); // masking the result to ensure only lower 12 bits
}

void read_DAC(){
	uint32_t ADC_value = Potentiometer_voltage();

	uint32_t DAC_value_read = ((float)ADC_value * 3300.0f) / 4095.0f; //
	//trace_printf("DAC value: %f\n", (float)DAC_value_read);

	DAC->DHR12R1 = DAC_value_read; // convert digital value into dac
}


void myGPIOB_Init()
{
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//	00: Input mode (reset state)
	//	01: General purpose output mode
	//	10: Alternate function mode
	//	11: Analog mode

        /* Configure PB2 (function generator) and PB3 (555 timer) as digital inputs */
        GPIOB->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);

        /* Configure PB8, PB9, and PB11 as digital outputs for the OLED control signals */
        GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER11);
        GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER11_0);

        /* Configure PB13 and PB15 for SPI alternate function (AF0) */
        GPIOB->MODER &= ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER15);
        GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1);

        /* Ensure no pull-up/pull-down for the configured pins */
        GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3 |
                          GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9 |
                          GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR13 |
                          GPIO_PUPDR_PUPDR15);

        /* Set default idle states for OLED control signals */
        GPIOB->ODR |= (GPIO_ODR_8 | GPIO_ODR_9 | GPIO_ODR_11);
}

void myGPIOC_Init()
{
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

        /* Configure PC8 and PC9 as outputs for the status LEDs */
        GPIOC->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
        GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);

        /* Ensure LEDs default to off */
        GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
        GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);
}


// LED Display Functions
void refresh_OLED( void )
{
	// Buffer size = at most 16 characters per PAGE + terminating '\0'
    	unsigned char Buffer[17];

    	snprintf( Buffer, sizeof( Buffer ), "  R: %5u Ohms  ", Res );

	oled_Write_Cmd(0xB3); //select page from 0xb0 to 0xb7
	oled_Write_Cmd(0x03); //fill every pixel on each page with 0x00, set lower column to 0
	oled_Write_Cmd(0x10); //select upper nibble for the column
	for(int i = 0; i < 18; i++){ // 0 - 17
		for(int j = 0; j < 8; j++){ // 0 - 7
			unsigned char result = Characters[Buffer[i]][j]; //Retrieve the graphical data for the current char, and assign to result
			oled_Write_Data(result); // send char pixel data to OLED
		}
	}

    snprintf( Buffer, sizeof( Buffer ), "  F: %5u Hz  ", Freq );

	oled_Write_Cmd(0xB5); //select page from 0xb0 to 0xb7
	oled_Write_Cmd(0x03); //fill every pixel on each page with 0x00, set lower column to 0
	oled_Write_Cmd(0x10); //select upper nibble for the column
	for(int i = 0; i < 18; i++){ // 0 - 17
		for(int j = 0; j < 8; j++){ // 0 - 7
			unsigned char result = Characters[Buffer[i]][j]; //Retrieve the graphical data for the current char, and assign to result
			oled_Write_Data(result); // send char pixel data to OLED
		}
	}

	if(inSig == 0){ // 555
		snprintf( Buffer, sizeof( Buffer ), "  Source: 555  " ); // show content when the source is 555
	}else{ //fg
		snprintf( Buffer, sizeof( Buffer ), "  Source: FG  " ); // show content when the source is FG
	}
	oled_Write_Cmd(0xB1); //select page from 0xb0 to 0xb7
	oled_Write_Cmd(0x03); //fill every pixel on each page with 0x00, set lower column to 0
	oled_Write_Cmd(0x10); //select upper nibble for the column
	for(int i = 0; i < 18; i++){ // 0 - 17
		for(int j = 0; j < 8; j++){ // 0 - 7
			unsigned char result = Characters[Buffer[i]][j]; //Retrieve the graphical data for the current char, and assign to result
			oled_Write_Data(result); // send char pixel data to OLED
		}
	}


	for(int i = 0; i < 10000; i++); // wait timer

}


void oled_Write_Cmd( unsigned char cmd ) // this function is to send command byte to OLED, initializing display
{
    GPIOB->ODR |= GPIO_ODR_8; // make PB8 = CS# = 1
    GPIOB->ODR &= ~(GPIO_ODR_9); // make PB9 = D/C# = 0
    GPIOB->ODR &= ~(GPIO_ODR_8); // make PB8 = CS# = 0
    oled_Write( cmd ); // write
    GPIOB->ODR |= GPIO_ODR_8; // make PB8 = CS# = 1
}

void oled_Write_Data( unsigned char data ) // this function is to send data byte to OLED, data represents the actual pixel (draw characters)
{
	GPIOB->ODR |= GPIO_ODR_8; // make PB8 = CS# = 1
	GPIOB->ODR |= GPIO_ODR_9; // make PB9 = D/C# = 1
	GPIOB->ODR &= ~(GPIO_ODR_8); // make PB8 = CS# = 0
    oled_Write( data );
    GPIOB->ODR |= GPIO_ODR_8; // make PB8 = CS# = 1
}


void oled_Write( unsigned char Value ) //send single byte over spi to oled
{

        while((SPI_Handle.Instance->SR & SPI_SR_TXE) == 0);

        HAL_SPI_Transmit( &SPI_Handle, &Value, 1, HAL_MAX_DELAY );

        while((SPI_Handle.Instance->SR & SPI_SR_TXE) == 0); // same as above
}


void oled_config( void )
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
        GPIOB->AFR[1] &= ~(0x0F << ((13 - 8) * 4));
        GPIOB->AFR[1] &= ~(0x0F << ((15 - 8) * 4));

        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

        SPI_Handle.Instance = SPI2;
	
	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.CRCPolynomial = 7;

// Initialize the SPI interface
    HAL_SPI_Init( &SPI_Handle );

// Enable the SPI
    __HAL_SPI_ENABLE( &SPI_Handle );


    //Reset LED display
    GPIOB->ODR &= ~(GPIO_ODR_11); // make pin PB11 = 0, wait for a few ms
	for(int i = 0; i < 1000; i++); //wait
    GPIOB->ODR |= GPIO_ODR_11; // make pin PB11 = 1, wait for a few ms
	for(int i = 0; i < 1000; i++); //wait

    // Send initialization commands to LED display
    for ( unsigned int i = 0; i < sizeof( oled_init_cmds ); i++ )
    {
        oled_Write_Cmd( oled_init_cmds[i] );
    }


    // Fill LED Display memory with zeros

    for (int PAGE = 0xB0; PAGE <= 0xB7; PAGE++){ // the loop goes thru 8 pages of the OLED
    	oled_Write_Cmd(PAGE); //select page from 0xb0 to 0xb7
    	oled_Write_Cmd(0x00); //fill every pixel on each page with 0x00, set lower column to 0
    	oled_Write_Cmd(0x10); //select upper nibble for the column
    	for (int SEG = 0; SEG < 128; SEG++){
    		oled_Write_Data( 0x00 ); // each page has 128 segments
    	}
    }

    trace_printf("OLED configured\n");
}


#pragma GCC diagnostic pop
