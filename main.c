#include <stdint.h>
#include <stm32f10x.h>
#include <stdbool.h>

int blink_on = 1;

void delay_us(uint32_t us) { //8 ticks/iteration
    __asm volatile (
        "push {r0}\r\n"
        "mov R0, %0\r\n"    //val = (9 * us) for 72Mhz
        "_loop:\r\n" //approx. 6ticks/iteration
            "cmp R0, #0\r\n"     //1
            "beq _exit\r\n"      //1 or 1+P (when condition is True)
            "sub R0, R0, #1\r\n" //1
            "nop\r\n" //1 allignment
            "b _loop\r\n" //1+P (pipeline refill) ~4 cycle
        "_exit:\r\n"
            "pop {r0}\r\n"
        :: "r"(9 * us) //for 72 Mhz
    );
}



/* Interrupt handler*/
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        if(GPIOC->ODR & GPIO_ODR_ODR13){
            GPIOC->ODR &= ~GPIO_ODR_ODR13;
        } else {
            GPIOC->ODR |= GPIO_ODR_ODR13;
        }

    //Clear interrupt flag
    TIM2->SR &= ~TIM_SR_UIF;
    }
}

/* EXTI0 Interrupt handler */
void EXTI0_IRQHandler() {
    EXTI->PR=EXTI_PR_PR0; // Сбрасываем прерывание
    if (blink_on == 1) {
        TIM2->CR1 &= ~TIM_CR1_CEN; // Stop timer
        blink_on = 0;
    } else {
        TIM2->CR1 |= TIM_CR1_CEN; // Start timer
        blink_on = 1;
    }
    delay_us(100);
}

int main(void) {
    // int i = 0;
    // int mask = 8; // 8 = 0b10000 = 0x8 = (1 << 4)
    // i = i | mask; // i |= mask;

    /* IO PORTS Configuration */
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // 0b10000=0x10
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); // GPIOC->CRH[23:20]=0000
    GPIOC->CRH |= GPIO_CRH_MODE13_0; // GPIOC->CRH[23:20]=0001

    // Port PB0 as Input
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; // 0b10000=0x10
    GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); // GPIOC->CRH[3:0]=0000
    GPIOB->CRL |= GPIO_CRL_CNF0_1; // GPIOB->CRH[3:0]=1000
    GPIOB->ODR |= GPIO_ODR_ODR0; //PB0 Internal pull-up resister

    // EXT10 Configuration
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB; // EXTI0=PB0
    EXTI->FTSR=EXTI_FTSR_TR0; // EXTI0 enable failing edge detection
    EXTI->IMR=EXTI_IMR_MR0; // Enable interrupt on EXTI0
    NVIC_SetPriority(EXTI0_IRQn, 0); //Делаем прерывание высокоприоритетным
    NVIC_EnableIRQ(EXTI0_IRQn); // Enable interrupt EXTI0

    /* TIM2 Configuration */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    TIM2->PSC = 2000;
    TIM2->ARR = 2000;
    TIM2->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ in NVIC
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer
    while (1) {
        __asm volatile ("nop");
    }

    while(1){

    }

    /*while(1){
        GPIOC->ODR &= ~GPIO_ODR_ODR13;
        delay_us(1000000);
        GPIOC->ODR |= GPIO_ODR_ODR13;
        delay_us(1000000);
    }*/

return 0;
}