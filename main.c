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

//track presses
uint32_t press_start_time = 0;
uint32_t press_duration   = 0;
uint8_t  button_pressed   = 0;

//store morse
char morse_code[100];
uint8_t morse_index = 0;

