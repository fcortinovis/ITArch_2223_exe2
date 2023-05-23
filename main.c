#define F_CPU 16000000L
#define sei() asm volatile("sei"::)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TASK_6




#ifdef TASK_1
//===============================//
// Task 1: Generate some outputs //
//===============================//
int main()
{
	DDRD = 0x00; // set all bits of DDRD to low -> inputs           
    DDRB = 0x00; // set all bits of DDRB to low -> inputs
	
//  The output LEDs are: D7, B0, B1; set the outputs accordingly
	DDRD  |= (1 << DDD7); // set DDD7 bit to 1 -> you have one output there
	DDRB  |= (1 << DDB0); // set DDB0 bit to 1 -> you have one output there
	DDRB  |= (1 << DDB1); // set DDB1 bit to 1 -> you have one output there

	PORTD |= (1 << PORTD7); // set the output to high
	PORTB |= (1 << PORTB0); // set the output to high
	PORTB |= (1 << PORTB1); // set the output to high

//  The reed sensors are connected to D2 and D3
	PORTD &=~(1 << PORTD2); // DDRD2 = 0 and PORTD2 = 1 -> D2 is an input with an external pull-down resistor
	PORTD &=~(1 << PORTD3); // DDRD3 = 0 and PORTD3 = 1 -> D3 is an input with an external pull-down resistor
	
	while(1)
	{
		PORTD |= (1 << PORTD7);
		PORTB |= (1 << PORTB0);
		PORTB |= (1 << PORTB1);

	//  Turn the LEDs on and off
		_delay_ms(300);
		PORTD &= ~(1 << PORTD7);
		_delay_ms(300);
		PORTB &= ~(1 << PORTB0);
		_delay_ms(300);
		PORTB &= ~(1 << PORTB1);
		_delay_ms(300);
	}
}
#endif

//
#ifdef TASK_2
//======================================================================//
// Task 2: generate digital output as a consequence of a digital input; //
// the inputs should be read using a polling strategy                   //
//======================================================================//
int main()
{
//  Configure output pins:
	DDRD = 0x00;          
	DDRB = 0x00;
	DDRD |= (1 << DDD7);
	DDRB |= (1 << DDB0);
	DDRB |= (1 << DDB1);
	
//  Make sure that the internal pull up resistors of the input pins (reed sensors) are disabled;
    PORTD &=~(1 << PORTD2); 
	PORTD &=~(1 << PORTD3);
	
	while(1)
	{
		if((PIND & (1 << PIND3)) > 0)
		{
		// The pin PIND3 of the register PIND has the state HIGH
			PORTD |= (1 << PORTD7);
		}
		else
		{
			PORTD &= ~(1 << PORTD7);
		}
	}
}	
#endif
//
#ifdef TASK_3
//======================================================//
// Task3: generate outputs with timer instead of delays //
//======================================================//

// First we need some global variables
volatile uint32_t iCounter = 0; // Timer overflow interrupt counter
volatile uint8_t flagB0 = 0;    // Flag associated to port B0
volatile uint8_t flagB1 = 0;    // Flag associated to port B1

void configureIO()
{
	DDRD = 0x00;
	DDRB = 0x00;

//  Configure the outputs:
	DDRD |= (1 << DDD7); // led A1 -> to be seen what effect it has on the ATmega on the BB-Board
	DDRB |= (1 << DDB0); // led A2 -> to be seen what effect it has on the ATmega on the BB-Board
	DDRB |= (1 << DDB1); // led A3 -> to be seen what effect it has on the ATmega on the BB-Board

//  Setting the bits of PortD and PortB LOW
	PORTD &= ~(1 << PORTD7);
	PORTB &= ~(1 << PORTB0);
	PORTB &= ~(1 << PORTB1);
	
//  Disable the pull up resistors of the input pins (reed sensors)
	PORTD &=~(1 << PORTD2);
	PORTD &=~(1 << PORTD3);
}

void configureTimer()
{
//	Timer Counter Control Register 0 A (TCCR0A) Set register to 0 (default)
//	For further information see ATmega328P manual
	TCCR0A = 0x00;

//	Timer Counter Control Register 0 B (TCCR0B) Set register to 0 (default)
	TCCR0B = 0x00;


//	Configure the prescaler:
	volatile const uint8_t prescaler = 5;
	configure_prescaler(prescaler)

//	TC0 Interrupt Mask Register (TIMSK0)
//	The TOIE bit activates the overflow interrupt
	*/
	TIMSK0 = 0x00;
	TIMSK0 |= (1 << TOIE0);
}

void configure_prescaler(uint8_t prescaler)
{
//	The three bits of TCCR0B CS00, CS01 and CS02 configure the prescaler:
//
//  | CS02 | CS01 | CS00 | Prescaler |
//	|   0  |   0  |   0  |  no clock |
//  |   0  |   0  |   1  |    1      |
//  |   0  |   1  |   0  |    8      |
//  |   0  |   1  |   1  |    64     |
//  |   1  |   0  |   0  |    256    |
//  |   1  |   0  |   1  |    1024   |
	switch (prescaler)
	{
		case 2: // prescaling factor 8
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			TCCR0B |= (1 << CS01); // Set CS01 to 1
			TCCR0B &=~(1 << CS00); // Set CS00 to 0
			break;
		case 3: // prescaling factor 64
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			TCCR0B |= (1 << CS01); // Set CS01 to 1
			TCCR0B |= (1 << CS00); // Set CS00 to 1
			break;
		case 4: // prescaling factor 256
			TCCR0B |= (1 << CS02); // Set CS02 to 1
			TCCR0B &=~(1 << CS01); // Set CS01 to 0
			TCCR0B &=~(1 << CS00); // Set CS00 to 0
			break;
		case 5: // prescaling factor 1024
			TCCR0B |= (1 << CS02); // Set CS02 to 1
			TCCR0B &=~(1 << CS01); // Set CS01 to 0
			TCCR0B |= (1 << CS00); // Set CS00 to 1
			break;
		default: // prescaling factor 1
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			TCCR0B &=~(1 << CS01); // Set CS01 to 0
			TCCR0B |= (1 << CS00); // Set CS00 to 1
			break;
	}
}

//	TIMER0_OVF_vect is the interrupt vector for an overflow of timer 0
//	ISR is the interrupt service routine
ISR(TIMER0_OVF_vect)
{
//  The clock of the ATmega328P is 16 MHz
//  The timer can save 8 bits (256 values) before it overflows

//	The overflow therefore occurs with one frequency
//	f = 16 MHz / 256 / (prescaler) = 62.5 kHz / (prescaler)

//	To define the time at which the event happens, calculation of number for iCounter is needed

	iCounter++;
	if((iCounter>=400) & (flagB0==0))
	{
		PORTB |=(1 << PORTB1);
		flagB0=1;
		iCounter=0;
	}

	if((iCounter>=400) & (flagB0==1))
	{
		PORTB &= ~(1 << PORTB1);
		flagB0=0;
		iCounter=0;
	}
}

int main()
{
	configureIO();
	configureTimer();
//  Set the interrupt register bits
	sei();	
	while(1)
    {
	}			
}
#endif
//
#ifdef TASK_4
//=================================================//
// Task4: generate outputs on external interrupts; //
//=================================================//
void configureIO()
{
//  Inputs:
	DDRD = 0x00;
	DDRB = 0x00;
//  Deactivate the pull up resistors:
	PORTD &=~(1 << PORTD2);
	PORTD &=~(1 << PORTD3);

//  Outputs:
	DDRD |= (1 << DDD7);
	DDRB |= (1 << DDB0);
	DDRB |= (1 << DDB1);

//  Set the outputs to be initially low:
	PORTD &= ~(1 << PORTD7);
	PORTB &= ~(1 << PORTB0);
	PORTB &= ~(1 << PORTB1);
}

void configureInterrupt()
{
	EICRA |= (1 << ISC10);
	EIMSK |= (1 << INT1);//enable interrupt 1 (associated to PD3, as described in the pinout diagram at page 3 of the manual)

	EICRA |= (1 << ISC00);
	EIMSK |= (1 << INT0);//enable interrupt 0 (associated to PD2, as described in the pinout diagram at page 3 of the manual)
}

ISR(INT1_vect)
{
	PORTB |=(1 << PORTB1);
}

ISR(INT0_vect)
{
	PORTB &= ~(1 << PORTB1);
}

int main()
{
	configureIO();
	configureInterrupt();
//  Set the interrupt register bits
	sei();		

	while(1)
	{
	}			
}
#endif
//
#ifdef TASK_5
//===================================================================//
// Task 5: hardware PWM generation (allows analog signal generation) //
//===================================================================//
void configureIO()
{	
	DDRD = 0x00;
	DDRB = 0x00;

//  Outputs:
	DDRD |= (1 << DDD6); // on PD6 we have the PWM generation to set the speed
	DDRD |= (1 << DDD7); // on PD7 we have the direction signal
	DDRB |= (1 << DDB1); // on PB1 we have the "enable" signal
	DDRB |= (1 << DDB0); // on PB0 we have the led A2 (otherwise unused)
	PORTB &= 0x00;

//  Inputs:
	PORTD &=~(1 << PORTD2); // pullup disabled on pin connected to reed sensor B1
	PORTD &=~(1 << PORTD3); // pullup disabled on pin connected to reed sensor B2
}

void configure_prescaler(uint8_t prescaler)
{
//	The three bits of TCCR0B CS00, CS01 and CS02 configure the prescaler:
	switch (prescaler)
	{
		case 2: // prescaling factor 8
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			TCCR0B |= (1 << CS01); // Set CS01 to 1
			TCCR0B &=~(1 << CS00); // Set CS00 to 0
			break;
		case 3: // prescaling factor 64
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			TCCR0B |= (1 << CS01); // Set CS01 to 1
			TCCR0B |= (1 << CS00); // Set CS00 to 1
			break;
		case 4: // prescaling factor 256
			TCCR0B |= (1 << CS02); // Set CS02 to 1
			TCCR0B &=~(1 << CS01); // Set CS01 to 0
			TCCR0B &=~(1 << CS00); // Set CS00 to 0
			break;
		case 5: // prescaling factor 1024
			TCCR0B |= (1 << CS02); // Set CS02 to 1
			TCCR0B &=~(1 << CS01); // Set CS01 to 0
			TCCR0B |= (1 << CS00); // Set CS00 to 1
			break;
		default: // prescaling factor 1
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			TCCR0B &=~(1 << CS01); // Set CS01 to 0
			TCCR0B |= (1 << CS00); // Set CS00 to 1
			break;
	}
}
void configurePWM()
{
//  The switching ratio can be specified using OCR0A.
//	0x80 (128) corresponds to 50%
//	0xFF (255) corresponds to 100%
	OCR0A = 200; //duty cycle (sets the value of the analog output)
	
	TIMSK0=0x00; // disable timer interrupts

//  Timer Counter Control Register 0 A set to default at first:
	TCCR0A=0x00;

//  The bit COM0A1 activates the PWM on output OC0A (pin PD6)):
	TCCR0A |= (1<<COM0A1);

//	Set the "mode of operation" to Fast PWM; for this we need both TCCR0A and TCCR0B:
	TCCR0B=0x00;
	TCCR0B &=~(1<<WGM02);
	TCCR0A |= (1<<WGM00 | 1<<WGM01);

//  As before we can use TCCR0B to set the prescaler value:
	configure_prescaler(1);
}

int main(void)
{
	configureIO();
	configurePWM();

	PORTD |= (1 << PORTD7); // on PD7 we have the direction signal
	PORTB |= (1 << PORTB1); // on PB1 we have the "enable" signal

    while (1) 
    {
    }
}
#endif
//
#ifdef TASK_6
//==============================================================//
// Task 6: combine previous tasks to control the rotating table //
//==============================================================//
void configureIO()
{
//	All bits of the DataDirectionRegister of Port D (DDRD) are set to LOW and are thus configured as inputs.
//	In binary notation, DDRD is now: 00000000
	DDRD = 0x00;
	DDRB = 0x00;

//  Outputs:
	DDRD |= (1 << DDD6); // on PD6 we have the PWM generation to set the speed
	DDRD |= (1 << DDD7); // on PD7 we have the direction signal
	DDRB |= (1 << DDB1); // on PB1 we have the "enable" signal
	DDRB |= (1 << DDB0); // on PB0 we have the led A2 (otherwise unused)
	PORTB &= 0x00;
//  Inputs:
	PORTD |= (1 << PORTD2); // pullup enabled on pin connected to reed sensor B1
	PORTD |= (1 << PORTD3); // pullup enabled on pin connected to reed sensor B2
}

void configurePWM()
{
//  The switching ratio can be specified using OCR0A. 0x80 (128) corresponds to 50%
	OCR0A = 64; //duty cycle (sets the value of the analog output)

//  Timer Counter Control Register 0 A. The bit COM0A1 activates the PWM (that is activates outpin OC0A-->PD6)
	TIMSK0=0x00;
	TCCR0A=0x00;
	TCCR0A |= (1<<COM0A1);

//  Timer Counter Control Register 0 A. Set the "mode of operation" to FastPWM
	TCCR0A |= (1<<WGM00 | 1<<WGM01); // see page 84 of the ATMega328P datasheet; Set OC0A (PD6) on compare-match when up-counting, clear when down-counting

//  Timer Counter Control Register 0 B. The frequency is the base frequency used without prescaler. That means 16 MHz; see page 87 ATMega328P datasheet.
	TCCR0B=0x00;
	//TCCR0B |= (1<<CS00);
	TCCR0B |= (1<<CS00 | 1<<CS01);	// sets the pre-scaler value.
}

void configureInterrupt()
{
//  Enable interrupt 0 (associated to PD2, see pinout diagram at page 3 of the manual)
	EIMSK |= (1 << INT0);
//  INT0 on rising edge
	EICRA |= (1 << ISC00);
	EICRA |= (1 << ISC01);


//  Enable interrupt 1 (associated to PD3, see pinout diagram at page 3 of the manual)
	EIMSK |= (1 << INT1);
//  INT1 on rising edge
	EICRA |= (1 << ISC10);
	EICRA |= (1 << ISC11);
}

ISR(INT1_vect)
{
//  PORTD |=(1 << PORTD7);
	PORTD &=~(1 << PORTD7);
}

ISR(INT0_vect)
{
	volatile static uint8_t cntr = 0;
    PORTD |=(1 << PORTD7);
//  PORTD &=~(1 << PORTD7);
	cntr += 1;
	if (cntr > 10)
	{
		PORTB &= ~(1 << PORTB1); // "disable"
	}
}

int main(void)
{
	configureIO();
	configurePWM();
	configureInterrupt();
	sei(); // enable interrupts
	PORTB |= (1 << PORTB1); // "enable"

    while (1) 
    {
    }
}
#endif
