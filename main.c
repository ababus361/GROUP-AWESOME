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


//milliseconds
#define SHORT_PRESS 200
#define LONG_PRESS 800

#define DOT '.'
#define DASH '-'
#define END ' '

//need to handle space between letters and space between words 

static char MorseLower[37] = {'a','b', 'c','d','e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
static char MorseUpper[37] = {'A','B', 'C','D','E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
static char * morse_table[37] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".--", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "..-", ".--", "-..-", "-.--", "--..", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.", "-----"};

//track presses
uint32_t press_start_time = 0;
uint32_t press_duration   = 0;
uint8_t  button_pressed   = 0;

//store morse
char morse_code[100];
uint8_t morse_index = 0;

void init_spi1_slow()
{
  RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

  RCC ->  APB2ENR |= RCC_APB2ENR_SPI1EN;

  GPIOB -> MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
  GPIOB -> AFR[0] |= 0x0;

  SPI1 -> CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; // SET MAX BAUD RATE
  SPI1 -> CR1 |= SPI_CR1_MSTR; // MASTER MODE
  SPI1 -> CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; // DATA SIZE 8 BITS

  SPI1 -> CR1 |= SPI_CR1_SSM;
  SPI1 -> CR1 |= SPI_CR1_SSI;

  SPI1 -> CR2 |= SPI_CR2_FRXTH;

  SPI1 -> CR1 |= SPI_CR1_SPE;
}

void enable_sdcard()
{

  GPIOB -> BRR |= GPIO_BRR_BR_2;
}

void disbale_sdcard()
{
  GPIOB -> BSRR |= GPIO_BSRR_BS_2;
}

void init_sdcard_io()
{
  init_spi1_slow();
  GPIOB -> MODER |= GPIO_MODER_MODER2_0;
  disbale_sdcard();
}

void sdcard_io_high_speed()
{
  SPI1 -> CR1 &= ~SPI_CR1_SPE;
  SPI1 -> CR1 |= SPI_CR1_BR_0; // DIVIDE SO CLOCK RATE IS 12 MHZ
  SPI1 -> CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi()
{
  RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

  
}

//things may need to change depending on where the button is plugged in 
void init_button_interrupt() { 
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOC->MODER &= ~GPIO_MODER_MODER0; //pc13

  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PA;
  EXTI->IMR |= EXTI_IMR_IM0;
  EXTI->RTSR |= EXTI_RTSR_TR0;

  NVIC_EnableIRQ(EXTI0_1_IRQn);
  SysTick_Config(SystemCoreClock/1000);
}

void Systick_Handler(void) { 
  if(button_pressed)  
  press_duration++;
}

void EXTI0_1_IRQHandler(void) { 
  if(EXTI->PR & EXTI_PR_PR0) {
    EXTI->PR |= EXTI_PR_PR0; 

    if(GPIOC->IDR & GPIO_IDR_0){
      button_pressed =1;
      press_duration = 0; 
    }
    else { 
      press_duration = 0; 

      if(press_duration < SHORT_PRESS) { 
        // makes sure the super short one is not used 
      }
      else if (press_duration < LONG_PRESS) {
        morse_code[morse_index++] = DOT; 
      }else { 
        morse_code[morse_index++] = DASH;
      }        
    }

  }

}

int main(void) { 
  init_spi1_slow();
  init_lcd_spi();
  init_sdcard_io();

    init_button_interrupt();

}