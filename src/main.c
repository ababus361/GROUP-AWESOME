/**
 ******************************************************************************
 * @file    main.c
 * @author  GROUP AWESOME
 * @date    Nov 1 2024
 * @brief   ECE 362 Mini project
 ******************************************************************************
 */

#include "stm32f0xx.h"
#include <string.h>
#include <stdio.h> 
#include "lcd.h"
#include "ff.h"
#include "diskio.h"
#include "sdcard.h"


#define WIDTH (320/2)
#define HEIGHT (240/2)

// milliseconds
#define SHORT_PRESS 200
#define LONG_PRESS 800

#define DOT '.'
#define DASH '-'
#define END ' '

// need to handle space between letters and space between words

static char MorseLower[36] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
static char MorseUpper[36] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
static char *morse_table[36] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.", "-----"};

uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};

// Global morse string
uint16_t morseString2[6] = {0x002};
//morseString2 = 0x002;
char morseString[6];
int strindex = 0;

void init_spi1_slow();
void nano_wait(unsigned int n);
void internal_clock();

// track presses
uint32_t press_start_time = 0;
uint32_t press_duration = 0;
uint8_t button_pressed = 0;

// store morse
char morse_code[100];
uint8_t morse_index = 0;


// CODE FROM LAB 6
void spi_cmd(unsigned int data) {
    while(!(SPI2 -> SR & SPI_SR_TXE)){}
    SPI2 -> DR = data;

}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
    
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(const char *string) {
    int i;
    spi_cmd(0x02); // move cursor to home position
    for(i = 0; string[i] != '\0'; i++){
        spi_data(string[i]);
    }

}
void spi1_display2(const char *string) {
    int i;
    spi_cmd(0xc0); // move cursor to second row
    for(i = 0; string[i] != '\0'; i++){
        spi_data(string[i]);
    }

}


FRESULT openSDCardFile(FATFS *FatFs, FIL *fil, char* filename) {
    // Open file, and return it's result code
    return f_open(fil, filename, FA_READ);
}

FRESULT closeSDCardFile(FATFS *FatFs, FIL *fil) {
    return f_close(fil); // Close file
}

void init_spi1_slow()
{
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;


  GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);


  SPI1->CR1 &= ~(SPI_CR1_BR);//|= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; // SET MAX BAUD RATE
  SPI1->CR1 |= SPI_CR1_MSTR;                               // MASTER MODE
  SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; // DATA SIZE 8 BITS

  SPI1->CR1 |= SPI_CR1_SSM;
  SPI1->CR1 |= SPI_CR1_SSI;

  SPI1->CR2 |= SPI_CR2_FRXTH;

  SPI1->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard()
{

  GPIOB->BRR |= GPIO_BRR_BR_2;
}

void disbale_sdcard()
{
  GPIOB->BSRR |= GPIO_BSRR_BS_2;
}

void init_sdcard_io()
{
  init_spi1_slow();
  GPIOB->MODER |= GPIO_MODER_MODER2_0;
  disbale_sdcard();
}

void sdcard_io_high_speed()
{
  SPI1->CR1 &= ~SPI_CR1_SPE;
  SPI1->CR1 |= SPI_CR1_BR_0; // DIVIDE SO CLOCK RATE IS 12 MHZ
  SPI1->CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi()
{
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  GPIOB -> MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0;
  init_spi1_slow();
  sdcard_io_high_speed();
}

// things may need to change depending on where the button is plugged in
void init_button_interrupt()
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER0; // PA0 input mode

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // EXTI0 for PA0
  //SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA; // EXTI2 FOR PA2

  EXTI-> RTSR |= EXTI_RTSR_TR0;
  EXTI-> IMR |= EXTI_IMR_IM0;

  //EXTI-> RTSR |= EXTI_RTSR_TR2;
  //EXTI-> IMR |= EXTI_IMR_IM2;

  NVIC_EnableIRQ(EXTI0_1_IRQn);
  //NVIC_EnableIRQ(EXTI2_3_IRQn);
  // SysTick_Config(SystemCoreClock/1000);
}

void init_rgb()
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODER1_0;
  GPIOA->BSRR |= GPIO_BSRR_BS_10 | GPIO_BSRR_BS_8 | GPIO_BSRR_BS_9;
  GPIOA->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;

}

void init_tim7()
{
  RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;

  TIM7 -> PSC = 48000 - 1;
  TIM7 -> ARR = 1000 - 1;

  TIM7 -> DIER |= TIM_DIER_UIE; // ENABLE TIMER INTERUPT 
  NVIC_EnableIRQ(TIM7_IRQn);

}

void init_spi2() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;
    GPIOB -> MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13);
    GPIOB -> MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1; // PB15, PB12, PB13 SET TO ALT MODE
    GPIOB -> AFR[1] = 0x00000000;

    RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2 -> CR1 &= ~SPI_CR1_SPE;
    SPI2 -> CR1 |= SPI_CR1_BR; //_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
    SPI2 -> CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_3; // 16 bit data size
    SPI2 -> CR1 |= SPI_CR1_MSTR; // MASTER ENABLE
    SPI2 -> CR2 |= SPI_CR2_SSOE; // SS OUTPUT ENABLE
    SPI2 -> CR2 |= SPI_CR2_NSSP; // NSSP ENABLE
    SPI2 -> CR2 |= SPI_CR2_TXDMAEN; // DMA OUTPUT ENABLE
    SPI2 -> CR1 |= SPI_CR1_SPE; // SPI ENABLE
}

void spi2_setup_dma(void) {
    RCC -> AHBENR |= RCC_AHBENR_DMAEN;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_EN; // turn off DMA1 CHANNEL3
    DMA1_Channel5 -> CMAR = (uint32_t)morseString2;
    DMA1_Channel5 -> CPAR = (uint32_t)&(SPI2 -> DR);
    DMA1_Channel5 -> CNDTR = 6;
    DMA1_Channel5 -> CCR |= DMA_CCR_DIR;
    DMA1_Channel5 -> CCR |= DMA_CCR_MINC;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_MSIZE_1;  // clear bits 0 and 1
    DMA1_Channel5 -> CCR &= ~DMA_CCR_PSIZE_1; 
    DMA1_Channel5 -> CCR |= DMA_CCR_MSIZE_0; // set bit 0 to 1
    DMA1_Channel5 -> CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5 -> CCR |= DMA_CCR_CIRC;
    DMA1_Channel5 -> CCR |= DMA_CCR_EN;
}

void togglexn(GPIO_TypeDef *port, int n)
{
  int32_t state;                // state of the specified pin
  state = port->ODR & (1 << n); // reads ODR value
  if (state)
  {
    port->BRR = (1 << n);
  }
  else
  {
    port->BSRR = (1 << n);
  }
}

int spaceCounter = 0;
int spaceCounterTripped = 0;

int xdim = 0;
int ydim = 0;

void morseSearch()
{
  int i = 0;

  while(i < 36)
  {
    if (strcmp(morseString, morse_table[i])==0)
    {
      //display(MorseUpper[i]);
      LCD_DrawChar(xdim,ydim,MAGENTA, BLACK, MorseUpper[i], 16, 0);
      togglexn(GPIOA, 1);
      nano_wait(1000000000);
      togglexn(GPIOA, 1);
      xdim += 8;
      if (xdim >= 240)
      {
        ydim += 16;
        xdim = 0;
      }
      spaceCounter = 0;
      spaceCounterTripped = 0;
    }
    i++;
  }



  /*
  convert morseString in nested loop
  - look through library
  - find index of mathcing string
  - use equal index in Alphabet library

  DMA transfer to display - TFT LCD
  */
}


void TIM7_IRQHandler()
{
  TIM7 -> SR &= ~TIM_SR_UIF;

  spaceCounter++;
  if((spaceCounter >= 3) && (spaceCounterTripped == 0))
  {
    // Insert space between morse words
    LCD_DrawChar(xdim,ydim,MAGENTA, BLACK, 32,16,0);
    xdim += 8;
    if (xdim >= 240)
    {
      ydim += 16;
      xdim = 0;
    }
    spaceCounter = 0;
    spaceCounterTripped = 1;
  }
  morseSearch();

  //morseString = NULL;
  for(int n = 0; n < 6; n++)
  {
    morseString[n] = NULL;
  }
  for(int n = 1; n < 6; n++)
  {
    morseString2[n] = NULL;
  }
  strindex = 0;
  /////////////////////////////////spi_cmd(0x01);
  ////////////////////////////////nano_wait(2000000);

  // togglexn(GPIOA, 5);
  // nano_wait(10000000);
  // togglexn(GPIOA, 5);
}

void Systick_Handler(void)
{
  if (button_pressed)
    press_duration++;
}




int main(void)
{
  init_lcd_spi();
  init_sdcard_io();
  LCD_Setup();
  init_spi2();
  spi1_init_oled();
  spi2_setup_dma();
  internal_clock();
  init_button_interrupt();
  init_rgb();
  init_tim7();
  spi1_init_oled();
  //spi1_display1("Hello again,");
}



void EXTI0_1_IRQHandler()
{
  EXTI->PR = EXTI_PR_PR0;

  TIM7 -> CR1 &= ~TIM_CR1_CEN;

  uint32_t timer = 0;

    

    while(((GPIOA -> IDR) & (GPIO_IDR_0)) != 0)
    {
      timer++;
      
    }
    
    //if (((GPIOA->IDR) & (GPIO_IDR_0)) != 0) {}
    if (timer <= 800000)
    {
      togglexn(GPIOA, 9);
      nano_wait(200000000);
      togglexn(GPIOA, 9);
      morseString[strindex] = '.';
      TIM7 -> CR1 |= TIM_CR1_CEN;
      TIM7 -> CNT = 0;
      //spi1_display1('.');
      morseString2[strindex + 1] = 0x200 + '.';
    }
    else
    {
      togglexn(GPIOA, 8);
      togglexn(GPIOA, 10);

      nano_wait(200000000);

      togglexn(GPIOA, 8);
      togglexn(GPIOA, 10);
      morseString[strindex] = '-';
      morseString2[strindex + 1] = 0x200 + '-';
      TIM7 -> CR1 |= TIM_CR1_CEN;
      TIM7 -> CNT = 0;
      //spi1_display1('-');
    }

    strindex++;
}

void nano_wait(unsigned int n)
{
  asm("        mov r0,%0\n"
      "repeat: sub r0,#83\n"
      "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
