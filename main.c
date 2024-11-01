/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 5 2024
  * @brief   ECE 362 Lab 1 template
  ******************************************************************************
*/


/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "ababus";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>

void initb();
void initc();
void setn(int32_t pin_num, int32_t val);
int32_t readpin(int32_t pin_num);
void buttons(void);
void keypad(void);
void autotest(void);
extern void internal_clock(void);
extern void nano_wait(unsigned int n);

int main(void) {
    internal_clock(); // do not comment!
    // Comment until most things have been implemented
    autotest();
    initb();
    initc();

    // uncomment one of the loops, below, when ready
    // while(1) {
    //   buttons();
    // }

    // while(1) {
    //   keypad();
    // }

    for(;;);
    
    return 0;
}

/**
 * @brief Init GPIO port B
 *        Pin 0: input
 *        Pin 4: input
 *        Pin 8-11: output
 *
 */
void initb() {
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER |= 0x00550000;
  GPIOB->MODER &= ~(0x00000303);
}

/**
 * @brief Init GPIO port C
 *        Pin 0-3: inputs with internal pull down resistors
 *        Pin 4-7: outputs
 *
 */
void initc() {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER |= 0x00005500;
  GPIOC->PUPDR |= 0x000000AA;
}

/**
 * @brief Set GPIO port B pin to some value
 *
 * @param pin_num: Pin number in GPIO B
 * @param val    : Pin value, if 0 then the
 *                 pin is set low, else set high
 */
void setn(int32_t pin_num, int32_t val) {
  if (val == 0) {
    GPIOB->BRR = (1 << pin_num);
  }
  else {
    GPIOB->BSRR = (1 << pin_num);
  }
}

/**
 * @brief Read GPIO port B pin values
 *
 * @param pin_num   : Pin number in GPIO B to be read
 * @return int32_t  : 1: the pin is high; 0: the pin is low
 */
int32_t readpin(int32_t pin_num) {
  if (GPIOB->IDR & (1 << pin_num)) {
    return 0x1;
  }
  else {
    return 0x0;
  }
}

/**
 * @brief Control LEDs with buttons
 *        Use PB0 value for PB8
 *        Use PB4 value for PB9
 *
 */
void buttons(void) {
  // Use the implemented subroutines
  // Read button input at PB0
  int32_t PB0 = readpin(0);
  // Put the PB0 value as output for PB8
  setn(8, PB0);
  // Read button input at PB4
  int32_t PB4 = readpin(4);
  // Put the PB4 value as output for PB9
  setn(9, PB4);
}

/**
 * @brief Control LEDs with keypad
 * 
 */
void keypad(void) {
  for (int i = 0; i < 4; i++) {
    GPIOC->ODR = (1 << (i + 4)); //pc4-pc7
    nano_wait(1000000); 
   
    int row = GPIOC->IDR & 0xF; //pco-pc3
    
    if (row & (1 << (i))) {
      setn(11 - i, 1); //on press
    } 
    else {
      setn(11 - i, 0); //off unpress
    }
  }
}
