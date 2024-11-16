#include <stdbool.h>

void InitializeGPIO();
void WriteMessage(char message[], int length);
void InitGPIO();
void InitializeUSART1();
unsigned int JoyStick();
unsigned int ADCData();

bool PD4State = false, PB5State = false, PA6State = false, PD2State = false, ClickState = false;
bool paused = false;
unsigned int rcvrd;
unsigned int C0Data;

void main() {
     InitGPIO();
     InitializeUSART1();
     
     for(;;) {

         if(((USART1_SR & (1<<5))== 0x20)){
             rcvrd = USART1_DR;
             if((rcvrd == 'p' || rcvrd == 'P') && paused == false){
                 WriteMessage("Paused", 6);
                 paused = true;
                 continue;
             }
             if((rcvrd == 'p' || rcvrd == 'P') && paused == true) {
                 WriteMessage("Unpaused", 8);
                 paused = false;
             }
         }

         if(paused) continue;

         switch(JoyStick()) {
         case 01:
             WriteMessage("UP Pressed",  10);
             break;
         case 02:
              WriteMessage("DOWN Pressed",  12);
             break;
         case 03:
              WriteMessage("RIGHT Pressed",  13);
             break;
         case 04:
              WriteMessage("LEFT Pressed",  12);
             break;
         case 05:
              WriteMessage("CLICK Pressed",  13);
             break;
         default:
             break;
         }
         
         GPIOD_ODR = ADCData() << 8;
         
     }

}

void InitializeUSART1(){ // Sub function which initializes the registers to enable USART1
RCC_APB2ENR |= 1; // Enable clock for Alt. Function. USART1 uses AF for PA9/PA10
AFIO_MAPR=0X0F000000; // Do not mask PA9 and PA10 (becaue we are using for USART)
RCC_APB2ENR |= 1<<2; // Enable clock for GPIOA
GPIOA_CRH &= ~(0xFF << 4); // Clear PA9, PA10
GPIOA_CRH |= (0x0B << 4); // USART1 Tx (PA9) output push-pull
GPIOA_CRH |= (0x04 << 8); // USART1 Rx (PA10) input floating
RCC_APB2ENR |= 1<<14; // enable clock for USART1
USART1_BRR=0X00000506; // Set baud rate to 56000
// Per data sheet (pg. 1010) USART1_CR1 consists of the following:
//13 12 11 10 9 8 7 6 5 4 3 2 1 0
//UE M WAKE PCE PS PEIE TXEIE TCIE RXNEIE IDLEIE TE RE RWU SBK
//rw rw rw rw rw rw rw rw rw rw rw rw rw rw
USART1_CR1 &= ~(1<<12); // Force 8 data bits. M bit is set to 0.
USART1_CR2 &= ~(3<<12); // Force 1 stop bit
USART1_CR3 &= ~(3<<8); // Force no flow control and no DMA for USART1
USART1_CR1 &= ~(3<<9); // Force no parity and no parity control
USART1_CR1 |= 3<<2; // RX, TX enable
//The following two instructions can also be used to enable RX and TX manually
//USART1_CR1.TE=1; //TX enable
//USART1_CR1.RE=1; //RX enable
USART1_CR1 |= 1<<13; // USART1 enable. This is done after configuration is complete
Delay_ms(100); // Wait for USART to complete configuration and enable. This is
// not always necessary, but good practice.
}

void InitGPIO() {
      RCC_APB2ENR |= 1 << 3;    // b
      RCC_APB2ENR |= 1 << 5;  //d
      RCC_APB2ENR |= 1 << 2;    //a
      RCC_APB2ENR |= 1 << 4;     //c
      RCC_APB2ENR |= 1 << 6;   //e
      RCC_APB2ENR |= 1 << 9 ;
      GPIOC_CRL &= ~(0xF << 0); // Configure PC0 as an Analog Input
      ADC1_SQR1 = (0b0000 << 20); // 1 conversion
      ADC1_SQR3 = 10; // Select Channel 10 as only one in conversion sequence
      ADC1_SMPR1 = 0b100; // Set sample time on channel 10
      ADC1_CR2 |= (0b111 << 17); // Set software start as external event forregular group conversion
      ADC1_CR2.ADON = 1;
      GPIOA_CRL = 0x44444444;
      GPIOD_CRL = 0X44444444;
      GPIOB_CRL = 0X44444444;
      GPIOC_CRH = 0x44444444;
      GPIOC_CRL = 0x33333333;
      GPIOE_CRH = 0x33333333;
      GPIOD_CRH = 0x33333333;
}

void WriteMessage(char message[], int length) {
    int j = 0;
    for(j = 0; j < length; j++) {
        while (! (USART1_SR & (1<<7)) == 0x80) {}
        USART1_DR = message[j];
        while(USART1_SR.TC == 0){}
    }
    while (! (USART1_SR & (1<<7)) == 0x80) {}
    USART1_DR = 0x0D;
    while(USART1_SR.TC == 0){}
    while (! (USART1_SR & (1<<7)) == 0x80) {}
    USART1_DR = 0x0A;
    while(USART1_SR.TC == 0){}
}

unsigned int JoyStick() {

    if(GPIOD_IDR.B4 == 0 && PD4State == false) {
         PD4State = true;
         return 1;
         //WriteMessage("UP Pressed",  10);
     }
     if(GPIOD_IDR.B4 == 1 && PD4State == true) {
         PD4State =  false;
     }

     if(GPIOB_IDR.B5 == 0 && PB5State == false) {
         PB5State = true;
         return 2;
         //WriteMessage("DOWN Pressed",  12);
     }
     if(GPIOB_IDR.B5 == 1 && PB5State == true) {
         PB5State =  false;
     }

     if(GPIOD_IDR.B2 == 0 && PD2State == false) {
         PD2State = true;
         return 4;
         //WriteMessage("LEFT Pressed",  12);
     }
     if(GPIOD_IDR.B2 == 1 && PD2State == true) {
         PD2State =  false;
     }

     if(GPIOA_IDR.B6 == 0 && PA6State == false) {
         PA6State = true;
         return 3;
         //WriteMessage("RIGHT Pressed",  13);
     }
     if(GPIOA_IDR.B6 == 1 && PA6State == true) {
         PA6State =  false;
     }

     if(GPIOC_IDR.B13 == 0 && ClickState == false) {
         ClickState = true;
         return 5;
         //WriteMessage("CLICK Pressed",  13);
     }
     if(GPIOC_IDR.B13 == 1 && ClickState == true) {
         ClickState =  false;
     }


     // LEDS
     GPIOE_ODR.B11 = PD4State || ClickState;
     GPIOE_ODR.B15 = PD4State || ClickState;

     GPIOE_ODR.B8 = PB5State || ClickState;
     GPIOE_ODR.B12 = PB5State || ClickState;

     GPIOE_ODR.B13 = PD2State || ClickState;
     GPIOE_ODR.B14 = PD2State || ClickState;

     GPIOE_ODR.B9 = PA6State || ClickState;
     GPIOE_ODR.B10 = PA6State || ClickState;
}

unsigned int ADCData(){
// Bit 20 is set to start conversion of an external channel, bit 22 starts the conversion
ADC1_CR2 |= (1 << 22) | (1 << 20);
while(!(ADC1_SR & 0b10)); // Wait until the ADC conversion has ended
return ADC1_DR; // Read value from data register. This also clears start bit
}