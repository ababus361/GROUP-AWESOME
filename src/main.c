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
void init_button_interrupt() 
{ 
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER0; // PA0 input mode

  RCC-> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // EXTI0 for PA0
  
  EXTI -> RTSR |= EXTI_RTSR_TR0;
  EXTI->IMR |= EXTI_IMR_IM0;
  
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  //SysTick_Config(SystemCoreClock/1000);
}

void init_rgb()
{
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA -> MODER |= GPIO_MODER_MODER5_0;
  //GPIOA ->                                                                                                                                
  GPIOA -> MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0;
  GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR10_1 | GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1;
}


void Systick_Handler(void) { 
  if(button_pressed)  
  press_duration++;
}

// void EXTI0_1_IRQHandler(void) { 
//   if(EXTI->PR & EXTI_PR_PR0) {
//     EXTI->PR |= EXTI_PR_PR0; 

//     if(GPIOC->IDR & GPIO_IDR_0){
//       button_pressed =1;
//       press_duration = 0; 
//     }
//     else { 
//       press_duration = 0; 

//       if(press_duration < SHORT_PRESS) { 
//         // makes sure the super short one is not used 
//       }
//       else if (press_duration < LONG_PRESS) {
//         morse_code[morse_index++] = DOT; 
//       }else { 
//         morse_code[morse_index++] = DASH;
//       }        
//     }

//   }

// }

int main(void) 
{ 
  init_spi1_slow();
  init_lcd_spi();
  init_sdcard_io();
  internal_clock();
  // init_usart5();
  // enable_tty_interrupt();
  // setup_tim1();
  init_button_interrupt();
  init_rgb();
  //setrgb();

}

// EXTI0
// init_button_interrupt();
// {
//     for(;;) {
//         // Breathe in...
//         for(float x=1; x<2400; x *= 1.1) {
//             TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
//             nano_wait(100000000);
//         }
//         // ...and out...
//         for(float x=2400; x>=1; x /= 1.1) {
//             TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
//             nano_wait(100000000);
//         }
//         // ...and start over.
//     }

// }
void togglexn(GPIO_TypeDef *port, int n) 
{
  int32_t state; // state of the specified pin
  state = port -> ODR & (1 << n); // reads ODR value
  if (state)
  {
    port -> BRR = (1 << n);
  }
  else
  {
    port -> BSRR = (1 << n);
  }
  
}

void EXTI0_1_IRQHandler()
{
  EXTI -> PR = EXTI_PR_PR0;
  togglexn(GPIOA, 9);
  togglexn(GPIOA,8);
  togglexn(GPIOA, 10);
}




void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

// void init_usart5() {
//     // TODO
//     //Enable the RCC clocks to GPIOC and GPIOD.
//     RCC -> AHBENR |= RCC_AHBENR_GPIOCEN; 
//     RCC -> AHBENR |= RCC_AHBENR_GPIODEN; 
//     //Enable the RCC clock to the USART5 peripheral.
//     RCC -> APB1ENR |= RCC_APB1ENR_USART5EN; 
    
//     //configure pin PC12 to be routed to USART5_TX & USART5_RX
//     //USART5_TX PC12
//     GPIOC -> MODER  &= ~(GPIO_MODER_MODER12); //CLear C 12
//     GPIOC -> MODER  |= GPIO_MODER_MODER12_1; //alternate (1) for 12
//     GPIOC->AFR[1] &= ~(0xF << (4 * (12 - 8)));
//     GPIOC->AFR[1] |= (2 << (4 * (12 - 8)));
//     //USART_RX PD2
//     GPIOD -> MODER  &= ~(GPIO_MODER_MODER2); //clear D 2
//     GPIOD -> MODER  |= GPIO_MODER_MODER2_1; //alternate (1) for 2
//     GPIOD->AFR[0] &= ~(0xF << (4 * 2));
//     GPIOD->AFR[0] |= (2 << (4 * 2));

//     //disable USART5 by turning off UE bit.
//     USART5 -> CR1 &= ~USART_CR1_UE; //off
//     //Set a word size of 8 bits.
//     USART5 -> CR1 &= ~USART_CR1_M; //M0 M1 clear
//     //Set 1 stop bit
//     USART5 -> CR2 &= ~USART_CR2_STOP;
//     //set no parity control
//     USART5 -> CR1 &= ~USART_CR1_PCE;
//     //Use 16x oversampling.
//     USART5 -> CR1 &= ~USART_CR1_OVER8; 
//     //Baud rate: 115200
//     //Family reference = SystemCoreClock
//     USART5 -> BRR = 48000000 / 115200;
//     //Enable the transmitter and the receiver by setting the TE and RE bits.
//     USART5 -> CR1 |= USART_CR1_TE; //TE
//     USART5 -> CR1 |= USART_CR1_RE; //RE
//     //Enable the USART.
//     USART5->CR1 |= USART_CR1_UE;
//     //wait for the TE and RE bits to be acknowledged by checking that TEACK and REACK
//     while(!(USART5->ISR & USART_ISR_TEACK));
//     while(!(USART5->ISR & USART_ISR_REACK));
// }



// int __io_putchar(int c) {
//     // TODO
//     if (c == '\n') {
//         while(!(USART5->ISR & USART_ISR_TXE));
//         USART5 -> TDR = '\r';
//     }
//     while(!(USART5->ISR & USART_ISR_TXE));
//     USART5->TDR = c;
//     return c;
// }

// int __io_getchar(void) {
//     while (!(USART5->ISR & USART_ISR_RXNE));
//     char c = USART5->RDR;
//     // TODO
//     if (c == '\r') {
//         c = '\n';
//     } 
//     __io_putchar(c);
//     return c;
// }



// #include <stdio.h>
// #include "fifo.h"
// #include "tty.h"

// #define FIFOSIZE 16
// char serfifo[FIFOSIZE];
// int seroffset = 0;

// TODO DMA data structures

// void enable_tty_interrupt(void) {
//     // TODO
//     USART5 -> CR1 |= USART_CR1_RXNEIE;
//     NVIC_EnableIRQ(USART3_8_IRQn);
//     USART5 -> CR3 |= USART_CR3_DMAR;

//     RCC -> AHBENR |= RCC_AHBENR_DMA2EN;
//     DMA2 -> CSELR |= DMA2_CSELR_CH2_USART5_RX;
//     DMA2_Channel2 -> CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off
    
//     DMA2_Channel2 -> CMAR = (uint32_t)serfifo;
//     DMA2_Channel2 -> CPAR = (uint32_t)&(USART5->RDR);
//     DMA2_Channel2 -> CNDTR = FIFOSIZE;
//     DMA2_Channel2 -> CCR &= ~(DMA_CCR_DIR);
//     DMA2_Channel2 -> CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
//     DMA2_Channel2 -> CCR |= DMA_CCR_MINC;
//     DMA2_Channel2 -> CCR &= ~DMA_CCR_PINC;
//     DMA2_Channel2 -> CCR |= DMA_CCR_CIRC;
//     DMA2_Channel2 -> CCR &= ~DMA_CCR_MEM2MEM;
//     DMA2_Channel2 -> CCR |= DMA_CCR_PL;

//     DMA2_Channel2 -> CCR |= DMA_CCR_EN;
// }

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
// char interrupt_getchar() {
//     // TODO
//     while (fifo_newline(&input_fifo) == 0) {
//         asm volatile ("wfi");  
//     }
//     return fifo_remove(&input_fifo);
// }

// int __io_putchar(int c) {
//     // TODO copy from STEP2
//     if (c == '\n') {
//         while(!(USART5->ISR & USART_ISR_TXE));
//         USART5 -> TDR = '\r';
//     }
//     while(!(USART5->ISR & USART_ISR_TXE));
//     USART5->TDR = c;
//     return c;
// }

// int __io_getchar(void) {
//     // TODO Use interrupt_getchar() instead of line_buffer_getchar()
//     return interrupt_getchar();
// }

// TODO Copy the content for the USART5 ISR here
// void USART3_8_IRQHandler(void) {
//     while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
//         if (!fifo_full(&input_fifo)) {
//             insert_echo_char(serfifo[seroffset]);
//         }
//         seroffset = (seroffset + 1) % sizeof serfifo;
//     }
// }
// // TODO Remember to look up for the proper name of the ISR function



// void setup_tim1(void) {
//     // Generally the steps are similar to those in setup_tim3
//     // except we will need to set the MOE bit in BDTR. 
//     // Be sure to do so ONLY after enabling the RCC clock to TIM1.
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
//     RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
//     //GPIOA->MODER &= ~(0xFF0000);
//     GPIOA->MODER |= 0xAA0000;

//     //GPIOA->AFR[1] &= ~(0xFFFF000);
//     GPIOA->AFR[1] |= 0x2222;

//     TIM1->BDTR |= TIM_BDTR_MOE;

//     TIM1->PSC = (1 - 1);
//     TIM1->ARR = (2400 - 1);

//     TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //8
//     TIM1 -> CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; //9
//     //TIM1 -> CCMR1 &= ~(TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC2M_0); //CLEAR 
    
//     TIM1 -> CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; //10
//     TIM1 -> CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; //11
//     //TIM1 -> CCMR2 &= ~(TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC4M_0);

//     TIM1->CCMR2 |= TIM_CCMR2_OC4PE; //ch 4

//     TIM1 -> CCER |= TIM_CCER_CC1E;
//     TIM1 -> CCER |= TIM_CCER_CC2E;
//     TIM1 -> CCER |= TIM_CCER_CC3E;
//     TIM1 -> CCER |= TIM_CCER_CC4E;
    
//     TIM1->CR1 |= TIM_CR1_CEN;
// }

//int getrgb(void);


// Helper function for you
// Accept a byte in BCD format and convert it to decimal
// uint8_t bcd2dec(uint8_t bcd) {
//     // Lower digit
//     uint8_t dec = bcd & 0xF;

//     // Higher digit
//     dec += 10 * (bcd >> 4);
//     return dec;
// }

// void setrgb(int rgb) {
//     int volume = 2400;

//     uint8_t b = bcd2dec(rgb & 0xFF);
//     uint8_t g = bcd2dec((rgb >> 8) & 0xFF);
//     uint8_t r = bcd2dec((rgb >> 16) & 0xFF);
    
//     int blue  = (100 - b) * volume / 100; 
//     int green = (100 - g) * volume / 100;
//     int red   = (100 - r) * volume / 100; 
//     // TODO: Assign values to TIM1->CCRx registers
//     // Remember these are all percentages
//     TIM1 -> CCR1 = red;
//     TIM1 -> CCR2 = green;
//     TIM1 -> CCR3 = blue;
//     // Also, LEDs are on when the corresponding PWM output is low
//     // so you might want to invert the numbers.
// }


