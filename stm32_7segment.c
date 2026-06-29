#include "stm32_7segment.h"

// --- HARDWARE CONFIGURATION ---
// If your display is Common Anode instead of Common Cathode, flip these!
#define DIGIT_ON  GPIO_PIN_RESET  // Pull LOW to activate digit ground
#define DIGIT_OFF GPIO_PIN_SET    // Pull HIGH to deactivate
#define SEG_ON    GPIO_PIN_SET    // Pull HIGH to light up segment
#define SEG_OFF   GPIO_PIN_RESET  // Pull LOW to turn off segment

// 0-9 Hex mapping for segments (A=bit0, B=bit1, ..., G=bit6)
// Assuming active HIGH segments
static const uint8_t digit_map[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Buffer holds the 4 digits to display
static uint8_t display_buffer[4] = {0, 0, 0, 0};
static uint8_t current_digit = 0;

void Display_SetTime(int seconds) {
    if (seconds > 9999) seconds = 9999;
    if (seconds < 0) seconds = 0;
    
    // Extract digits (e.g., 0030)
    display_buffer[0] = seconds / 1000;
    display_buffer[1] = (seconds / 100) % 10;
    display_buffer[2] = (seconds / 10) % 10;
    display_buffer[3] = seconds % 10;
}

// Helper to write bits to GPIO ports matching the physical pinout
static void write_segments(uint8_t val) {
    // Segments A-G mapped to PC6 through PC12
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  (val & 0x01) ? SEG_ON : SEG_OFF); // A
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  (val & 0x02) ? SEG_ON : SEG_OFF); // B
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  (val & 0x04) ? SEG_ON : SEG_OFF); // C
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  (val & 0x08) ? SEG_ON : SEG_OFF); // D
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (val & 0x10) ? SEG_ON : SEG_OFF); // E
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (val & 0x20) ? SEG_ON : SEG_OFF); // F
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, (val & 0x40) ? SEG_ON : SEG_OFF); // G

    // Decimal Point mapped to PD2
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, SEG_OFF); // DP kept off for countdown timer
}

// Put this in your TIM6 PeriodElapsedCallback (~2ms interrupt)
void Display_Multiplex_ISR(void) {
    // 1. Turn OFF all digits to prevent ghosting (Digits mapped to PB6-PB9)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, DIGIT_OFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, DIGIT_OFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, DIGIT_OFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, DIGIT_OFF);

    // 2. Load segment data for the current digit
    // Optional: Blank leading zeros by checking if display_buffer[current_digit] == 0 
    // and current_digit < 2, but for countdowns, showing "0030" is fine.
    write_segments(digit_map[display_buffer[current_digit]]);

    // 3. Turn ON the current digit
    switch (current_digit) {
        case 0: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, DIGIT_ON); break;
        case 1: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, DIGIT_ON); break;
        case 2: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, DIGIT_ON); break;
        case 3: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, DIGIT_ON); break;
    }

    // 4. Move to next digit
    current_digit = (current_digit + 1) % 4;
}
