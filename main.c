#include <stdio.h>
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

// init functions
void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myGPIOB_Init(void);

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

uint32_t timerTriggered = 0;
uint32_t inSig = 0;


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

	myGPIOA_Init();	// initialize I/O port PA
 	myGPIOB_Init(); // initalize PB
	myDAC_Init(); // initialize DAC
	myTIM2_Init(); //Initialize timer TIM2
	myADC_Init(); // initialize ADC
	oled_config(); // configure the screen
	refresh_OLED(); // clock the first tick of the screen so that it is up before any other interupts
	myEXTI_Init(); //initialize EXTI


	while (1)
	{
		read_DAC(); // read dac (cont. updates the dac value as well)

		refresh_OLED(); // refresh screen
	}
	return 0;
}



void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/* Configure PA2 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
	/* Ensure no pull-up/pull-down for PA2 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);

	/* Configure PA4 as analog output */
	GPIOA->MODER |= 0b001100000000; // 0011 0000 0000 change bit 8 and 9
	/* Ensure no pull-up/pull-down for PA4 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	/* Configure PA5 as analog input */
	GPIOA->MODER |= 0b110000000000; // 1100 0000 0000 change bit 10 and 11
	/* Ensure no pull-up/pull-down for PA5 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
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
	/* Map EXTI2 each EXTI */
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI2);
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0);
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);

	/* all EXTI line interrupts: set rising-edge trigger */
	EXTI->RTSR |= (EXTI_RTSR_TR0);
	EXTI->RTSR |= (EXTI_RTSR_TR1);
	EXTI->RTSR |= (EXTI_RTSR_TR2);

	/* Unmask interrupts from each EXTI line */
	EXTI->IMR |= (EXTI_IMR_MR0);
	EXTI->IMR |= (EXTI_IMR_MR1);
	EXTI->IMR |= (EXTI_IMR_MR2);

	/* Assign EXTI2 interrupt priority = 1 in NVIC */
	NVIC_SetPriority(EXTI2_3_IRQn,1);

	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI2_3_IRQn);

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn,0);

	/* Enable EXTI1 interrupts in NVIC */
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

void EXTI0_1_IRQHandler(){
	// Declare/initialize your local variables here...
	double freq;
	uint32_t count;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0) {
		
		//when EXTI1 when inSig = 0 */

		// If this is the first edge:
		//	Clear count register (TIM2->CNT).
		//	Start timer (TIM2->CR1).
		//  Else (this is the second edge):
		//	Stop timer (TIM2->CR1).
		//	Read out count register (TIM2->CNT).
		//	Calculate signal period and frequency.
		//	Print calculated values to the console.
		if((EXTI->PR & EXTI_PR_PR1) != 0){ // check interupt 1
			if(timerTriggered == 0){ // first edge
				timerTriggered = 1; // swap
				count = 0; // clear count

				TIM2->CNT = 0; // clear count register

				TIM2->CR1 |= TIM_CR1_CEN;; // start timer
			}else{ // second edge
				timerTriggered = 0; // swap

				TIM2->CR1 &= ~(TIM_CR1_CEN); // stop timer

				count = TIM2->CNT; // update count

				freq = ((double)SystemCoreClock)/((double)count); // record freq

				trace_printf("Source: 555\n");
				trace_printf("The Frequency is: %f Hz\n", freq);
				trace_printf("The Resistance is: %f ohms\n\n", (float)Potentiometer_resistance());

				Freq = (unsigned int)freq; //transmist to the global vresion for our display
				Res = (unsigned int)Potentiometer_resistance(); // transmit to global version for our display
			}
		}
		if((EXTI->PR & EXTI_PR_PR0) != 0){ // is button pressed?
			while((GPIOA->IDR & GPIO_IDR_0) != 0){} // wait for button
			EXTI->IMR &= ~(EXTI_IMR_MR1); // disable this interupt (1)
			EXTI->IMR &= ~(EXTI_IMR_MR0); // clear and set intterupt 0
			EXTI->IMR |= (EXTI_IMR_MR0);
		}

		// Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;
	}


	if ((EXTI->PR & EXTI_PR_PR0) != 0) { // intterupt 0
		/* 
  		If inSig = 0, then: let inSig = 1, disable
		EXTI1 interrupts, enable EXTI2 interrupts
		Else: let inSig = 0, disable EXTI2 interrupts,
		enable EXTI1 interrupts 
  		*/

		EXTI->PR |= 0x1; // clear interupt pending flag for exti 0



//		inSig = 0 is 555
//		inSig = 1 is fg
		if(inSig == 0){
			inSig = 1; // swap
			EXTI->IMR &= ~(1 << 1);//disable exti1
			EXTI->IMR |= (1 << 2); //enable exti2
		}else{
			inSig = 0; // swap
			EXTI->IMR &= ~(1 << 2);//disable exti2
			EXTI->IMR |= (1 << 1);//enable exti1
		}
	}

}

void EXTI2_3_IRQHandler()
{
	double freq;
	uint32_t count;
	/* Check if EXTI2 interrupt pending flag is indeed set */
	// this reads off function generator
	if ((EXTI->PR & EXTI_PR_PR2) != 0){

		// same method as above
			if((EXTI->PR & EXTI_PR_PR2) != 0){
				if(timerTriggered == 0){
					timerTriggered = 1;
					count = 0;

					TIM2->CNT = 0;

					TIM2->CR1 |= TIM_CR1_CEN;;
				}else{
					timerTriggered = 0;

					TIM2->CR1 &= ~(TIM_CR1_CEN);

					count = TIM2->CNT;

					freq = ((double)SystemCoreClock)/((double)count); // freq=1/period

					trace_printf("Source: Function Generator\n");
					trace_printf("The Frequency is: %f Hz\n", freq);
					trace_printf("The Resistance is: %f ohms\n\n", (float)Potentiometer_resistance());

					Freq = (unsigned int)freq; //transmist to the global vresion for our display
					Res = (unsigned int)Potentiometer_resistance(); // transmit to glboal version for our display
				}
			}

		// button press in this inttereupt
		if((EXTI->PR & EXTI_PR_PR0) != 0){ // is button pressed?
			while((GPIOA->IDR & GPIO_IDR_0) != 0){} // wait for button
			EXTI->IMR &= ~(EXTI_IMR_MR2); // disable this interupt
			EXTI->IMR &= ~(EXTI_IMR_MR0); // clear adn set button
			EXTI->IMR |= (EXTI_IMR_MR0);
		}

		EXTI->PR |= EXTI_PR_PR2;

	}
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


	ADC1->CHSELR = ADC_CHSELR_CHSEL5; // select CHSEL5 for ADC channel 5

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

	/* Configure PB3 as alternate */
	GPIOB->MODER &= ~(GPIO_MODER_MODER3_0); // clear low bit
	GPIOB->MODER |= (GPIO_MODER_MODER3_1); // set high bit
	// this works as we get 00(from clear) to 10(from set)
	// apply to below

	/* Configure PB4 as ouput */
	GPIOB->MODER &= ~(GPIO_MODER_MODER4_1); // clear high bit
	GPIOB->MODER |= (GPIO_MODER_MODER4_0); // set low bit
	//00 -> 01

	/* Configure PB5 as alternate */
	GPIOB->MODER &= ~(GPIO_MODER_MODER5_0);
	GPIOB->MODER |= (GPIO_MODER_MODER5_1);

	/* Configure PB6 as ouput */
	GPIOB->MODER &= ~(GPIO_MODER_MODER6_1);
	GPIOB->MODER |= (GPIO_MODER_MODER6_0);

	/* Configure PB7 as ouput */
	GPIOB->MODER &= ~(GPIO_MODER_MODER7_1);
	GPIOB->MODER |= (GPIO_MODER_MODER7_0);

	/* Ensure no pull-up/pull-down for each */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
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
    GPIOB->ODR |= GPIO_ODR_6; // make PB6 = CS# = 1
    GPIOB->ODR &= ~(GPIO_ODR_7); // make PB7 = D/C# = 0
    GPIOB->ODR &= ~(GPIO_ODR_6); // make PB6 = CS# = 0
    oled_Write( cmd ); // write
    GPIOB->ODR |= GPIO_ODR_6; // make PB6 = CS# = 1
}

void oled_Write_Data( unsigned char data ) // this function is to send data byte to OLED, data represents the actual pixel (draw characters)
{
	GPIOB->ODR |= GPIO_ODR_6; // make PB6 = CS# = 1
	GPIOB->ODR |= GPIO_ODR_7; // make PB7 = D/C# = 1
	GPIOB->ODR &= ~(GPIO_ODR_6); // make PB6 = CS# = 0
    oled_Write( data );
    GPIOB->ODR |= GPIO_ODR_6; // make PB6 = CS# = 1
}


void oled_Write( unsigned char Value ) //send single byte over spi to oled
{

	while((SPI1->SR & SPI_SR_TXE) == 0);

  	HAL_SPI_Transmit( &SPI_Handle, &Value, 1, HAL_MAX_DELAY );

	while((SPI1->SR & SPI_SR_TXE) == 0); // same as above
}


void oled_config( void )
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	GPIOB->AFR[1] &= ~(0x0F << (3 * 4)); // clear AFR bit in PB3
	GPIOB->AFR[1] &= ~(0x0F << (5 * 4)); // PB5
	
	GPIOB->AFR[1] |= (0x0 << (3 * 4)); // set AF0 to bit in PB3
	GPIOB->AFR[1] |= (0x0 << (5 * 4)); // PB5
	
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	SPI_Handle.Instance = SPI1;
	
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
    GPIOB->ODR &= ~(GPIO_ODR_4); // make pin PB4 = 0, wait for a few ms
	for(int i = 0; i < 1000; i++); //wait
    GPIOB->ODR |= GPIO_ODR_4; // make pin PB4 = 1, wait for a few ms
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