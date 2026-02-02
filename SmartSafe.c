#include "stm32f446xx.h"
#include <stdint.h>

/* ================= CONFIG ================= */
#define LCD_ADDR       (0x27 << 1)
#define LCD_BACKLIGHT  0x08

#define PASS_LEN 4

#define SERVO_CLOSE_PULSE 1000
#define SERVO_OPEN_PULSE  2000

#define BUZZER_PORT GPIOD
#define BUZZER_PIN  2

#define LED_PORTA_MASK ((1<<8)|(1<<9)|(1<<10)|(1<<11))
#define LED_PORTB_MASK ((1<<12)|(1<<13)|(1<<14)|(1<<15))

char keyMap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

/* ================= PROTOTYPES ================= */
void SystemClock_HSI(void);
void SysTick_Init(void);
void Delay_ms(uint32_t ms);

/* I2C + LCD */
void GPIO_I2C_Init(void);
void I2C1_Init(void);
void I2C_Start(void);
void I2C_Write(uint8_t data);
void I2C_Stop(void);
void LCD_Init(void);
void LCD_Command(uint8_t cmd);
void LCD_Data(char data);
void LCD_String(char *str);
void LCD_Write4(uint8_t data, uint8_t rs);
void LCD_Clear_Line2(void);

/* Keypad */
void GPIO_Keypad_Init(void);
char Keypad_GetKey(void);

/* Servo */
void Servo_Init(void);
void Servo_Open(void);
void Servo_Close(void);

/* Buzzer */
void Buzzer_Init(void);
void Buzzer_Buzz(uint32_t ms);

/* LEDs */
void LED_Init(void);
void LED_All_On(void);
void LED_All_Off(void);
void LED_Blink(uint8_t times);
void LED_Blink_With_Buzzer(uint32_t duration_ms);

/* ================= MAIN ================= */
int main(void)
{
    char input[PASS_LEN];
    uint8_t index = 0;

    SystemClock_HSI();
    SysTick_Init();

    GPIO_I2C_Init();
    I2C1_Init();
    GPIO_Keypad_Init();
    Servo_Init();
    Buzzer_Init();
    LED_Init();
    LCD_Init();

    LCD_Command(0x01);
    Delay_ms(5);
    LCD_String("ENTER PASSWORD");
    LCD_Command(0xC0);

    while (1)
    {
        char key = Keypad_GetKey();
        if (!key) continue;

        if (key == 'C')
        {
            Servo_Close();
            LED_All_Off();
            LCD_Clear_Line2();
            LCD_String("LOCKED");
            Buzzer_Buzz(200);
            Delay_ms(1000);
            LCD_Clear_Line2();
            index = 0;
            continue;
        }

        if (key >= '0' && key <= '9' && index < PASS_LEN)
        {
            input[index++] = key;
            LCD_Data('*');
        }
        else if (key == 'A' && index > 0)
        {
            index--;
            LCD_Command(0x10);
            LCD_Data(' ');
            LCD_Command(0x10);
        }
        else if (key == 'B')
        {
            LCD_Clear_Line2();
            index = 0;
        }

        if (index == PASS_LEN)
        {
            LCD_Clear_Line2();

            if (input[0]=='1' && input[1]=='2' &&
                input[2]=='3' && input[3]=='4')
            {
                LCD_String("CORRECT");
                Servo_Open();
                LED_All_On();
                Buzzer_Buzz(200);
                Delay_ms(100);
                Buzzer_Buzz(200);
            }
            else
            {
                LCD_String("INCORRECT");
                LED_Blink_With_Buzzer(1500);   // ?? LED + buzzer together
            }

            Delay_ms(2000);
            LCD_Clear_Line2();
            index = 0;
        }

        Delay_ms(300);
    }
}

/* ================= CLOCK ================= */
void SystemClock_HSI(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
}

/* ================= SYSTICK ================= */
void SysTick_Init(void)
{
    SysTick->LOAD = 16000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5;
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
        while (!(SysTick->CTRL & (1 << 16)));
}

/* ================= BUZZER ================= */
void Buzzer_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    BUZZER_PORT->MODER &= ~(3 << (BUZZER_PIN * 2));
    BUZZER_PORT->MODER |=  (1 << (BUZZER_PIN * 2));
}

void Buzzer_Buzz(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t++)
    {
        BUZZER_PORT->ODR ^= (1 << BUZZER_PIN);
        for (volatile int i = 0; i < 200; i++);
    }
    BUZZER_PORT->ODR &= ~(1 << BUZZER_PIN);
}

/* ================= LEDs ================= */
void LED_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    GPIOA->MODER &= ~(0xFF << 16);
    GPIOA->MODER |=  (0x55 << 16);

    GPIOB->MODER &= ~(0xFF << 24);
    GPIOB->MODER |=  (0x55 << 24);

    LED_All_Off();
}

void LED_All_On(void)
{
    GPIOA->ODR |= LED_PORTA_MASK;
    GPIOB->ODR |= LED_PORTB_MASK;
}

void LED_All_Off(void)
{
    GPIOA->ODR &= ~LED_PORTA_MASK;
    GPIOB->ODR &= ~LED_PORTB_MASK;
}

void LED_Blink(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        LED_All_On();
        Delay_ms(300);
        LED_All_Off();
        Delay_ms(300);
    }
}

/* ?? LEDs + buzzer together */
void LED_Blink_With_Buzzer(uint32_t duration_ms)
{
    for (uint32_t t = 0; t < duration_ms; t++)
    {
        LED_All_On();
        BUZZER_PORT->ODR ^= (1 << BUZZER_PIN);
        for (volatile int i = 0; i < 200; i++);
    }

    LED_All_Off();
    BUZZER_PORT->ODR &= ~(1 << BUZZER_PIN);
}

/* ================= SERVO ================= */
void Servo_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOA->MODER &= ~(3 << 0);
    GPIOA->MODER |=  (2 << 0);
    GPIOA->AFR[0] |=  (1 << 0);

    TIM2->PSC  = 16 - 1;
    TIM2->ARR  = 20000 - 1;
    TIM2->CCR1 = SERVO_CLOSE_PULSE;

    TIM2->CCMR1 |= (6 << 4);
    TIM2->CCER  |= 1;
    TIM2->CR1   |= 1;
}

void Servo_Open(void)
{
    TIM2->CCR1 = SERVO_OPEN_PULSE;
}

void Servo_Close(void)
{
    TIM2->CCR1 = SERVO_CLOSE_PULSE;
}

/* ================= I2C ================= */
void GPIO_I2C_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3<<16)|(3<<18));
    GPIOB->MODER |=  (2<<16)|(2<<18);

    GPIOB->OTYPER |= (1<<8)|(1<<9);
    GPIOB->PUPDR |= (1<<16)|(1<<18);
    GPIOB->AFR[1] |= (4<<0)|(4<<4);
}

void I2C1_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = LCD_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
}

void I2C_Write(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2C_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

/* ================= LCD ================= */
void LCD_Write4(uint8_t data, uint8_t rs)
{
    uint8_t high = data | LCD_BACKLIGHT | rs | 0x04;
    uint8_t low  = data | LCD_BACKLIGHT | rs;

    I2C_Start();
    I2C_Write(high);
    Delay_ms(2);
    I2C_Write(low);
    I2C_Stop();
}

void LCD_Command(uint8_t cmd)
{
    LCD_Write4(cmd & 0xF0, 0);
    LCD_Write4(cmd << 4, 0);
    Delay_ms(2);
}

void LCD_Data(char data)
{
    LCD_Write4(data & 0xF0, 1);
    LCD_Write4(data << 4, 1);
}

void LCD_String(char *str)
{
    while (*str) LCD_Data(*str++);
}

void LCD_Clear_Line2(void)
{
    LCD_Command(0xC0);
    for (int i = 0; i < 16; i++) LCD_Data(' ');
    LCD_Command(0xC0);
}

void LCD_Init(void)
{
    Delay_ms(50);
    LCD_Write4(0x30,0); Delay_ms(5);
    LCD_Write4(0x30,0); Delay_ms(2);
    LCD_Write4(0x30,0); Delay_ms(2);
    LCD_Write4(0x20,0); Delay_ms(5);
    LCD_Command(0x28);
    LCD_Command(0x08);
    LCD_Command(0x01);
    Delay_ms(3);
    LCD_Command(0x06);
    LCD_Command(0x0C);
}

/* ================= KEYPAD ================= */
void GPIO_Keypad_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOC->MODER &= ~(0xFF);
    GPIOC->MODER |=  0x55;

    GPIOC->PUPDR &= ~(0xFF << 8);
    GPIOC->PUPDR |=  (0x55 << 8);

    GPIOC->ODR |= 0x0F;
}

char Keypad_GetKey(void)
{
    for (int row = 0; row < 4; row++)
    {
        GPIOC->ODR |= 0x0F;
        GPIOC->ODR &= ~(1 << row);
        Delay_ms(1);

        for (int col = 0; col < 4; col++)
        {
            if (!(GPIOC->IDR & (1 << (col + 4))))
            {
                while (!(GPIOC->IDR & (1 << (col + 4))));
                return keyMap[row][col];
            }
        }
    }
    return 0;
}