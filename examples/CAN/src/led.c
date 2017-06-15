//*************************************************************************************

#include "stm32f4xx.h"
#include "led.h"

#define GREEN   (1ull << 12)
#define ORANGE  (1ull << 13)
#define RED     (1ull << 14)
#define BLUE    (1ull << 15)

const unsigned long leds[] = {GREEN, ORANGE, RED, BLUE};

//*************************************************************************************

void LED_Init (void) 

{

  RCC->AHB1ENR  |= ((1UL <<  3) );         /* Enable GPIOD clock                */

  GPIOD->MODER    &= ~((3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15)  );   /* PD.12..15 is output               */

  GPIOD->MODER    |=  ((1UL << 2*12) |
                       (1UL << 2*13) | 
                       (1UL << 2*14) | 
                       (1UL << 2*15)  ); 

  GPIOD->OTYPER   &= ~((1UL <<   12) |
                       (1UL <<   13) |
                       (1UL <<   14) |
                       (1UL <<   15)  );   /* PD.12..15 is output Push-Pull     */

  GPIOD->OSPEEDR  &= ~((3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15) );   /* PD.12..15 is 50MHz Fast Speed     */

  GPIOD->OSPEEDR  |=  ((2UL << 2*12) |
                       (2UL << 2*13) | 
                       (2UL << 2*14) | 
                       (2UL << 2*15)  ); 

  GPIOD->PUPDR    &= ~((3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15)  );   /* PD.12..15 is Pull up              */

  GPIOD->PUPDR    |=  ((1UL << 2*12) |
                       (1UL << 2*13) | 
                       (1UL << 2*14) | 
                       (1UL << 2*15)  ); 
}

//*************************************************************************************

void resetLED (unsigned int bit) 

{
   GPIOD->BSRRH =  leds[bit];
}

//*************************************************************************************

void setLED (unsigned int bit)

{
   GPIOD->BSRRL =  leds[bit];
}

//*************************************************************************************

void LED_Out(unsigned int value) {
  int bit;
  // const int max = sizeof(leds)/sizeof(unsigned long);

  for (bit = 0; bit < LED_NUM; bit++) 
  {
    (value & (1 << bit)) ? setLED(bit) : resetLED(bit);
  }

}
