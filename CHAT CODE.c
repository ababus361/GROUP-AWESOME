#include "stm32f0xx_hal.h"
#include "fatfs.h"  // Include for SD card handling
#include "tft.h"    // Include for TFT LCD handling (define SPI interface in this header)

// Global Variables
volatile uint8_t button_pressed = 0;
volatile uint32_t press_start_time = 0;
char morse_buffer[64];  // Buffer to store morse code sequence
char text_buffer[256];  // Buffer to store converted text
FATFS fs;  // File system for SD card
FIL file;  // File object

// Morse code table (example for A-F)
const char* morse_table[6] = { ".-", "-...", "-.-.", "-..", ".", "..-." };
//static char MorseLower[37] = {'a','b', 'c','d','e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
//static char MorseUpper[37] = {'A','B', 'C','D','E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
//static String morse_table[37] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".--", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "..-", ".--", "-..-", "-.--", "--..", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.", "-----"};

// Function Declarations
void SystemClock_Config(void);
void GPIO_Init(void);
void Timer_Init(void);
void DMA_Init(void);
void SPI_Init(void);
void SD_Card_Init(void);
void Save_Text_to_SD(void);
void Convert_Morse_To_Text(void);

// Main function
int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    Timer_Init();
    DMA_Init();
    SPI_Init();
    SD_Card_Init();
    
    TFT_Init();  // Initialize TFT screen (assuming a function is in the tft.h file)

    while (1) {
        if (button_pressed) {
            uint32_t press_duration = HAL_GetTick() - press_start_time;
            if (press_duration > 500) {  // Long press threshold
                strcat(morse_buffer, "-");
            } else {
                strcat(morse_buffer, ".");
            }
            button_pressed = 0;  // Reset button press
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Toggle LED for feedback
        }

        // Example check if buffer is full or a pause is detected for letter completion
        if (strlen(morse_buffer) >= 6 || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
            Convert_Morse_To_Text();  // Convert and display current Morse input
            Save_Text_to_SD();        // Save current text to SD card
            memset(morse_buffer, 0, sizeof(morse_buffer));  // Clear buffer
        }
    }
}

// Function Definitions

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure Button Input
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure LED Output
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable External Interrupt for Button
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void Timer_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM_HandleTypeDef htim3;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 48000 - 1;  // 1 ms tick
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1;  // 1 second overflow
    HAL_TIM_Base_Init(&htim3);
    HAL_TIM_Base_Start_IT(&htim3);
}

void DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    // Configure DMA for data transfer from buffer (if needed)
    // DMA settings depend on your specific use case and data size.
}

void SPI_Init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    SPI_HandleTypeDef hspi1;
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi1);
}

void SD_Card_Init(void) {
    f_mount(&fs, "", 1);
}

void Convert_Morse_To_Text(void) {
    // Simple example to map Morse buffer to English letters
    for (int i = 0; i < 6; i++) {
        if (strcmp(morse_buffer, morse_table[i]) == 0) {
            text_buffer[strlen(text_buffer)] = 'A' + i;
            break;
        }
    }

    // Display text_buffer on TFT
    TFT_DisplayText(text_buffer);
}

void Save_Text_to_SD(void) {
    if (f_open(&file, "MORSELOG.TXT", FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
        f_write(&file, text_buffer, strlen(text_buffer), NULL);
        f_close(&file);
    }
}

// Interrupt Handlers

void EXTI0_1_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
        button_pressed = 1;
        press_start_time = HAL_GetTick();
    }
}

void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
}

// HAL Timer Callback for further timing actions if necessary
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // Timer overflow actions (if required)
    }
}
