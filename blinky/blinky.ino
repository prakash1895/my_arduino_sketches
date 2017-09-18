#include<avr/io.h>
#include<util/delay.h>

int main()

{
  DDRB |= 0b11111111;
  while(1)
  {
    PORTB|=0b11111111;
    _delay_ms(200);

    PORTB&= ~(0b11111111);
    _delay_ms(200);

    return(0);
  }
}

