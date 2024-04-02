#include <stdint.h>
#include "tm4c123gh6pm.h"
void initUartT();
void initC6();
void initTimer();
void Timer0Handler();
void GPIOC_Handler();
void UART_Tx (char data);
void initB();
void PortC_keypad_init();
void GPIOB_Handler();
//output pin
#define GPIODATA_A (*((volatile unsigned long *) 0x40004010))    //base address of gpio port A offset with pin 2 masking bit
#define GPIODIR_A (*((volatile unsigned long *) 0x40004400))   //address of pin direction Port A
#define GPIODEN_A (*((volatile unsigned long *) 0x4000451C))   //address of digital enable Port A
#define GPIOA_AFSEL (*((volatile unsigned long *) 0x40004420)) //address of afsel for PORT A
#define GPIOA_PCTL (*((volatile unsigned long *) 0x4000452C))
#define RCGCGPIO (*((volatile unsigned long *) 0x400FE608)) //Clocks set bit 0 and 1 for port A

//Port C  input
#define GPIOBASE_C (*((volatile unsigned long *) 0x40006000))    //base address of gpio port C
#define GPIODATA_C6_R (*((volatile unsigned long *) 0x40006100)) //port C pin 6 read
#define GPIODIR_C (*((volatile unsigned long *) 0x40006400))   //address of pin direction Port C
#define GPIODEN_C (*((volatile unsigned long *) 0x4000651C))   //address of digital enable Port C
#define GPIODEN_PORTC_PIN6 0x040 // mask for PC6

//Port B out
#define GPIO_PORTB_0_7       (*((volatile unsigned long *)0x400053FC))

#define UART0 (*((volatile unsigned long *) 0x4000C000))
#define UARTFR (*((volatile unsigned long *) 0x4000C018)) //address for containing receive or transmit bit
#define UARTDR (*((volatile unsigned long *) 0x4000C000)) //
#define RCGCUART (*((volatile unsigned long *) 0x400FE618)) //address for uart clock
#define UART0_IBRD (*((volatile unsigned long *) 0x4000C024)) //address for uart0 ibrd
#define UART0_FBRD (*((volatile unsigned long *) 0x4000C028)) //address for UART0 fbrd
#define UARTLCRH (*((volatile unsigned long *) 0x4000C02C)) //address for UART0 Line Control
#define UART0CC (*((volatile unsigned long *) 0x4000CFC8)) //address for UART0 clock config
#define UARTCTL (*((volatile unsigned long *) 0x4000C030)) //address for enabling UART0

//timer
//#define SYSCTL_RCGCTIMER_R      (*((volatile unsigned long *)0x400FE604))
//#define TIMER0_CTL_R            (*((volatile unsigned long *)0x4003000C))
//#define TIMER0_CFG_R            (*((volatile unsigned long *)0x40030000))
//#define TIMER0_TAMR_R           (*((volatile unsigned long *)0x40030004))
//#define TIMER0_TAILR_R          (*((volatile unsigned long *)0x40030028))
//#define TIMER0_ICR_R            (*((volatile unsigned long *)0x40030024))
//#define TIMER0_IMR_R            (*((volatile unsigned long *)0x40030018))
//#define TIMER0_TAPR_R           (*((volatile unsigned long *)0x40030038))
//#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
//#define NVIC_EN0_INT_M          0xFFFFFFFF  // Interrupt Enable

/**
 * main.c
 */
void initTimer(){
    SYSCTL_RCGCTIMER_R |= 0x01;  //enable timer0
    TIMER0_CTL_R &= (~(1<<0)); //disable timer
    TIMER0_CFG_R = 0x4; //16 bit timer
    TIMER0_TAMR_R = 0x2; //setting as periodic timer and counts down
    TIMER0_TAILR_R = 64000;  //count down from this number
    TIMER0_ICR_R |= 0x01; //clear interrupt status flags
    TIMER0_TAPR_R = 0xFF; //set prescaler
    TIMER0_CTL_R |= 0x01; //enable timer
    TIMER0_IMR_R |= 0x01; //enable interrupt for timer A
    //enable bit 19 in nvic
    NVIC_EN0_R |= (1<<19); //200000 NVIC_EN0_INT_M

}
void Timer0Handler(){
    GPIODATA_A = 0x7;
    volatile uint32_t ui32Loop;
    for(ui32Loop = 0;ui32Loop<10000;ui32Loop++){

    }
    TIMER0_ICR_R = 0x01;

}

/*
void GPIOC_Handler(){
    //C4 C5 C6 C7
    //UART transmit '$' on rising edge
    if((GPIO_PORTC_MIS_R & (1<<6)) == (1<<6)){ //check if interrupt from pin 6
        UART_Tx('$');
        volatile uint32_t ui32Loop;
        for(ui32Loop = 0;ui32Loop<200000;ui32Loop++){

        }
    }

    //clear interrupt flag
    GPIO_PORTC_ICR_R |= (0xF0);

    //may need to insert some delay

}
*/

void init_A2(){
    GPIODEN_A = 0x04;
    GPIODIR_A = 0x04;
}

void initUartT(){
    GPIODIR_A |= 0x03;
    GPIODEN_A |= 0x03;
    GPIOA_AFSEL = 0x03;
    GPIOA_PCTL = 0x11;
}
void initC6(){
    GPIODEN_C = GPIODEN_PORTC_PIN6; //digital enable PC6
    //GPIODIR_C &= ~GPIODEN_PORTC_PIN6; // clear direction bit for PC6 for output with bitwise AND, inverted mask
    GPIODIR_C = 0x0;
    GPIO_PORTC_PDR_R |= (0xF0); //Pull-down resistors
    /*
    //configure GPIOC6 interrupt sense
    GPIO_PORTC_IS_R &= ~(0xF0);
    // clear IBE register so not both edges are detected
    GPIO_PORTC_IBE_R &= ~(0xF0);
    // set IEV for rising edge trigger
    GPIO_PORTC_IEV_R |= (0xF0);
    //clear prior interrupt
    GPIO_PORTC_ICR_R |= (0xF0);
    //unmask interrupt for PC6
    GPIO_PORTC_IM_R |= (0xF0);

    //NVIC config
    NVIC_EN0_R |= (1<<2); // Port C
    //interrupt priority (maybe?)
     */

}
void initB(){
    GPIO_PORTB_DEN_R |= 0x7F;
    GPIO_PORTB_DIR_R |= 0x7F;
    //GPIO_PORTB_0_7 = 0x1E;


    GPIO_PORTB_DEN_R |= (1<<6); //digital enable PC6
    GPIO_PORTB_DIR_R &= ~(1<<6);

    //configure GPIOB6 interrupt sense
    GPIO_PORTB_IS_R &= ~(1<<6);
    // clear IBE register so not both edges are detected
    GPIO_PORTB_IBE_R &= ~(1<<6);
    // set IEV for rising edge trigger
    GPIO_PORTB_IEV_R |= (1<<6);
    //clear prior interrupt
    GPIO_PORTB_ICR_R |= (1<<6);
    //unmask interrupt for PB6
    GPIO_PORTB_IM_R |= (1<<6);

    //NVIC config
    NVIC_EN0_R |= (1<<1); // Port B
    //interrupt priority (maybe?)

}

void GPIOB_Handler(){
    //UART transmit '$' on rising edge
    if((GPIO_PORTB_MIS_R & (1<<6)) == (1<<6)){ //check if interrupt from pin 6
        UART_Tx('$');
    }
    volatile uint32_t ui32Loop;
    for(ui32Loop = 0;ui32Loop<200000;ui32Loop++){}
    //clear interrupt flag
    GPIO_PORTB_ICR_R |= (1<<6);

    //may need to insert some delay

}

void UART_Tx (char data){
    while(UARTFR & 0x20); //busy waiting while full FIFO bit set
    UARTDR = data;
}

char UART_Rx(char c){
    char data;
    while(UARTFR & 0x10);
    data = UARTDR;
    return ((unsigned char) data);

}
void PortC_keypad_init(){
    GPIO_PORTC_CR_R  |= 0xF0; //Allow settings for all pins of PORTC
    GPIO_PORTC_DEN_R |= 0xF0; //Digital enable all portC pins
}
int main(void)
{
    volatile uint32_t ui32Loop;
    char symbol[4][4] = {
      {'*','1','4','7'},
      {'0','2','5','8'},
      {'d','a','b','c'},
      {'#','3','6','9'}
    };
    //init clocks
    RCGCGPIO = 0x00000007; //enable clock for port A and port C B
    RCGCUART = 0x01; //start UART clock
    for(ui32Loop = 0;ui32Loop<2000;ui32Loop++){

    }
    //init Port A for output
    init_A2();

    //init port C for input
    initC6();
    PortC_keypad_init();
    //init port b for output
    initB();
    //initialize UART
    initUartT();
    //disable uarten in UARTCTL
    UARTCTL = 0x0;
    //set ibrd
    UART0_IBRD = 104; //9600 baud
    //set FBRD
    UART0_FBRD = 11; //9600 baud
    //set line control
    UARTLCRH = 0x60;
    UART0CC = 0x0;
    UARTCTL = 0x301;
    initTimer();
    while(1){
        GPIODATA_A = 0x3;
        volatile uint32_t i;
        volatile uint32_t j;
        for(i = 0; i < 4; i++)    //Scan columns loop
        {
            GPIO_PORTB_0_7 = (1 << i+1);
            for(ui32Loop = 0;ui32Loop<5;ui32Loop++){

            }
            for(j = 0; j < 4; j++)  //Scan rows
            {
                if((GPIO_PORTC_DATA_R & 0xF0) & (1 << j+4)){
                    UART_Tx(symbol[j][i]);
                }
                for(ui32Loop = 0;ui32Loop<18000;ui32Loop++){

                }
            }
        }
    }
}
