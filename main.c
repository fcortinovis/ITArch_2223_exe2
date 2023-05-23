#define F_CPU 16000000L
#define sei() asm volatile("sei"::)

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
//  Configure the outputs:
void configureTimer()
	For further information see ATmega328P manual
	TCCR0B = 0x00;

//	Configure the prescaler:
	volatile const uint8_t prescaler = 5;
	configure_prescaler(prescaler)
	/*
	The TOIE bit activates the overflow interrupt
void configure_prescaler(uint8_t prescaler)
{
//	The three bits of TCCR0B CS00, CS01 and CS02 configure the prescaler:
	switch (prescaler)
	{
		case 1: // prescaling factor 1
		case 2: // prescaling factor 8
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			break;
		case 3: // prescaling factor 64
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
}
/*
	ISR is the interrupt service routine
ISR(TIMER0_OVF_vect)
	The timer can save 8 bits (256 values) before it overflows

	The overflow therefore occurs with one frequency
	f = 16 MHz / 256 / (prescaler) = 62.5 kHz / (prescaler)

	To define the time at which the event happens, calculation of number for iCounter is needed
	if((iCounter>=400) & (flagB0==1))
int main()
#endif
//
#ifdef TASK_4
//=================================================//
// Task4: generate outputs on external interrupts; //
void configureInterrupt()
	EICRA |= (1 << ISC00);
ISR(INT1_vect)
ISR(INT0_vect)
int main()
	while(1)
#endif
//
#ifdef TASK_5
//===================================================================//
// Task 5: hardware PWM generation (allows analog signal generation) //
//===================================================================//
void configureIO()
	In binary notation, DDRD is now: 00000000
	/*	
	In binary notation, DDRD is now: 10000000
//  Outputs:
{
//	The three bits of TCCR0B CS00, CS01 and CS02 configure the prescaler:
	switch (prescaler)
	{
		case 1: // prescaling factor 1
		case 2: // prescaling factor 8
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
			break;
		case 3: // prescaling factor 64
			TCCR0B &=~(1 << CS02); // Set CS02 to 0
}
void configurePWM()
	0x80 (128) corresponds to 50%
int main(void)
{
	PORTD |= (1 << PORTD7); // on PD7 we have the direction signal
    while (1) 
//
#ifdef TASK_6
//==============================================================//
// Task 6: combine previous tasks to control the rotating table //
//==============================================================//
void configureIO()
	In binary notation, DDRD is now: 00000000
//  Outputs:

void configurePWM()
//  Timer Counter Control Register 0 A. The bit COM0A1 activates the PWM (that is activates outpin OC0A-->PD6)
//  Timer Counter Control Register 0 A. Set the "mode of operation" to FastPWM
//  Timer Counter Control Register 0 B. The frequency is the base frequency used without prescaler. That means 16 MHz; see page 87 ATMega328P datasheet.

void configureInterrupt()

ISR(INT1_vect)
{
ISR(INT0_vect)
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