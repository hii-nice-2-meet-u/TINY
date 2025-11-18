
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
 *
 *		File		:	tiny.h
 *		Release		:	v0.1
 *
 *		Date	:	Tue 18 Nov 2025
 *		Author	:	hii-nice-2-meet-u
 *
 *
 *	[ Contact Information ]
 *	G-mail		:	0x0.whitecat@gmail.com
 *	Discord		:	@hii_nice.2.meet.u
 *	Github		:	https://github.com/hii-nice-2-meet-u
 *
 */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef INC_TINY_H
#define INC_TINY_H

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <Arduino.h>

#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#define DELAY_BACKWARD_COMPATIBLE
#include <util/atomic.h>
#include <util/delay.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//. Pin Definitions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define __TINY_PIN_PH_A 4
#define __TINY_PIN_EN_A 5

#define __TINY_PIN_EN_B 6
#define __TINY_PIN_PH_B 7

#define __TINY_PIN_PWM_A __TINY_PIN_EN_A
#define __TINY_PIN_DIR_A __TINY_PIN_PH_A

#define __TINY_PIN_PWM_B __TINY_PIN_EN_B
#define __TINY_PIN_DIR_B __TINY_PIN_PH_B

#define __TINY_PIN_SW_A 8

#define __TINY_PIN_LED_A __TINY_PIN_SW_A
#define __TINY_PIN_LED_B 3

#define __TINY_PIN_SPI_SCLK 13
#define __TINY_PIN_SPI_MISO 12
#define __TINY_PIN_SPI_MOSI 11
#define __TINY_PIN_SPI_SS0  10
#define __TINY_PIN_SPI_SS1  9

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//. TypeDef Definitions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

typedef int8_t  TYPE__i8;
typedef int16_t TYPE__i16;
typedef int32_t TYPE__i32;
typedef int64_t TYPE__i64;

typedef uint8_t  TYPE__u8;
typedef uint16_t TYPE__u16;
typedef uint32_t TYPE__u32;
typedef uint64_t TYPE__u64;

typedef float  TYPE__f32;
typedef double TYPE__f64;

#define i8  TYPE__i8
#define i16 TYPE__i16
#define i32 TYPE__i32
#define i64 TYPE__i64

#define u8  TYPE__u8
#define u16 TYPE__u16
#define u32 TYPE__u32
#define u64 TYPE__u64

#define f32 TYPE__f32
#define f64 TYPE__f64

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//` MotorControl Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//$ Function initialize Motor
void _init_Motor(void)
{
	//- Reset Timer/Counter 0
	/// Reset Timer0 Counter to exactly zero
	TCNT0 = 0;

	//- Set PWM Output Register PD5 & PD6
	/// set Timer0 to Fast PWM Mode 			: (1 << WGM01) | (1 <<
	/// WGM00) non-inverting mode on OC0A and OC0B		: (1 << COM0A1)
	/// | (1 << COM0B1)
	TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);

#ifdef ___DISABLE_TIMER0_INTERRUPT___
	//- Disable Timer0 Interrupt
	/// Disable Timer0 overflow interrupt		: Clear (1 << TOIE0) in TIMSK0
	TIMSK0 = 0;

	//- Set prescaler to 001 ( Clk/1 no Pre-scaling ) [ ~62.5 kHz ]
	/// Prescaler - clk/1 ( no prescaling )		: (1 << CS00)
	TCCR0B = (1 << CS00);

#else
	//- Set prescaler to 011 ( Clk/64 from Pre-scaler ) [ ~976.5625 Hz ]
	/// Prescaler - clk/64 ( from Pre-scaler )	: (1 << CS01) | (1 << CS00)
	TCCR0B = (1 << CS01) | (1 << CS00);

#endif

	//- Set Pin as Output
	/// MotorPin	:	PD4, PD5, PD6, PD7 -> output
	DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);

	//- Set Output LOW
	/// MotorPin	:	PD4, PD5, PD6, PD7 => LOW
	PORTD &= ~((1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));

	//? Set Output PWM to 0
	OCR0B = 0; //< | MotorA | PMW A | PD5 |
	OCR0A = 0; //> | MotorB | PMW B | PD6 |
}

//* ================================================================

//$ Control Motor
void Motor(i16 LSp, i16 RSp)
{
	volatile u8 TEMP_PORTD = PORTD & 0b01101111;

	/// Set Side - A
	if (LSp < 0)
	{
		LSp = -LSp;
		TEMP_PORTD |= (1 << PIN7);
	}

	/// Set Side - B
	if (RSp < 0)
	{
		RSp = -RSp;
		TEMP_PORTD |= (1 << PIN4);
	}

	//- Set Pin DIR
	PORTD = TEMP_PORTD;

	//- Set Pin PWM
	OCR0A = (LSp >> 8) ? 255 : (u8)LSp; //> MotorA | PMW A | PD6 |
	OCR0B = (RSp >> 8) ? 255 : (u8)RSp; //< MotorB | PMW B | PD5 |
}

//* ================================================================

//+ Control Motor LEFT & RIGHT in same Speed
static inline void Motor(i16 Sp) { Motor(Sp, Sp); }

//+ Control Motor in Time duration
static inline void Motor(i16 LSp, i16 RSp, const f64 duration)
{
	Motor(LSp, RSp);
	_delay_ms(duration);
}

//+ Stop PWM Signal
static inline void Motor(void)
{
	OCR0B = 0; //< MotorA | PMW A | PD5 |
	OCR0A = 0; //> MotorB | PMW B | PD6 |
}

//* ================================================================

#define MotorStop(void) Motor(void)
#define ao(void)        Motor(void)
#define motor           Motor

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//` Function Math [ Fast Absolute ]
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//$ Math function Get Absolute of value by pretty fast - 8 bits
i8 __abs_8b(i8 n)
{
	i8 TEMP = n >> (7);
	return ((n + TEMP) ^ TEMP);
}

//$ Math function Get Absolute of value by pretty fast - 16 bits
i16 __abs_16b(i16 n)
{
	i16 TEMP = n >> (15);
	return ((n + TEMP) ^ TEMP);
}

//$ Math function Get Absolute of value by pretty fast - 32 bits
i32 __abs_32b(i32 n)
{
	i32 TEMP = n >> (31);
	return ((n + TEMP) ^ TEMP);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//` Timer Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef ___DISABLE_TIMER0_INTERRUPT___

u32 __TIMER1_COUNTER__ = 0;

//* ================================================================

//\ " Pause "
/// Function to Pause Time Counter
#define Timer__Pause(void) TIMSK1 &= ~(1 << TOIE1)

//\ " Resume "
/// Function to Resume Time Counter
#define Timer__Resume(void) TIMSK1 |= (1 << TOIE1)

//* ================================================================

//$ Function Initialize Timer
void _init_Timer(void)
{
	//- Reset Timer/Counter 1
	/// Reset Timer1 Counter to exactly zero
	TCNT1 = 0;

	//- Set Timer 1 to CTC (Clear Timer on Compare Match) mode
	/// set Timer1 to CTC Mode 					: clear (1 <<
	/// WGM11) & (1 << WGM10) in TCCR1A
	TCCR1A = 0;

	//- Set prescaler to 011 ( Clk/64 from Pre-scaler )
	/// Prescaler - clk/64 ( from Pre-scaler )	: (1 << CS01) | (1 << CS00)
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

	//- Set compare match value for 1ms interval
	/// Calculation: OCR1A = (F_CPU / prescaler / 1000) - 1
	OCR1A = 249;

	TIMSK1 = (1 << OCIE1A);

	//. Reset Timer Counter
	__TIMER1_COUNTER__ = 0;

	//. Enable Timer1 compare match A interrupt
	Timer__Resume();

	//. Enable global interrupts
	sei();
}

//* ================================================================

//$ Timer 1 compare match interrupt handler
ISR(TIMER1_COMPA_vect) { __TIMER1_COUNTER__++; }

//* ================================================================

//\ " Get "
/// Function to get the current Time Counter
u32 Timer__Get(void)
{
	u32 TEMP;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { TEMP = __TIMER1_COUNTER__; }
	return TEMP;
}

//\ " Set "
/// Function to Set Time Counter
void Timer__Set(u32 TimeSet)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { __TIMER1_COUNTER__ = TimeSet; }
}

//\ " Reset "
/// Function to Reset Time Counter
void Timer__Reset(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { __TIMER1_COUNTER__ = 0; }
}

//* ================================================================

#else

//* ================================================================

#define Timer__Get(void) millis(void)

#define Timer__Pause(void)  TIMSK0 &= ~(1 << TOIE0)
#define Timer__Resume(void) TIMSK0 |= (1 << TOIE0)

#define Timer__Set(TimeSet)
#define Timer__Reset(void)

//* ================================================================

#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//` SPI Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef ___ENABLE_SPI0___

//* ================================================================

#define SPI0_DDR  DDRB
#define SPI0_SCLK DDB5
#define SPI0_MISO DDB4
#define SPI0_MOSI DDB3
#define SPI0__SS0 DDB2
#define SPI0__SS1 DDB1

//* ================================================================

//$ Function initialize SPI0
void _init_SPI0(void)
{
	//- Set Pin as Output
	/// SPI Pins	:	PB2, PB3, PB4, PB5 -> output
	SPI0_DDR |= (1 << SPI0__SS0) | (1 << SPI0__SS1) | (1 << SPI0_MOSI) | (1 << SPI0_SCLK);

	//- enable SPI, set as master, and clock to fosc/8
	/// SPE			: Enable SPI
	/// MSTR		: Set as Master
	/// SPR0, SPI2X : Set clock to fosc/8
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	SPSR = (1 << SPI2X);
}

//* ================================================================

//+ Transmit Byte - SPI0
void SPI0_Tx(u8 data)
{
	SPDR = data;
	while (!(SPSR & (1 << SPIF)))
		;
}

//+ Receive Byte - SPI0
u8 SPI0_Rx(void)
{
	SPDR = 0xFF;
	while (!(SPSR & (1 << SPIF)))
		;

	return SPDR;
}

//+ Transmit & Receive Byte - SPI0
u8 SPI0_TxRx(u8 data)
{
	SPDR = data;
	while (!(SPSR & (1 << SPIF)))
		;

	return SPDR;
}

//* ================================================================

#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//` Switch and LED Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//$ Function initialize Switch and LED Functions
void _init_SwitchAndLED(void)
{
	//- Set Pin Switch as Input
	/// Switch A : PB0
	DDRB &= ~((1 << DDB0));

	//- Set Pin LED & IO as Input
	/// LED B : PD3 / IO : PD2
	DDRD &= ~((1 << DDD2) | (1 << DDD3));
}

//* ================================================================
//. Switch A
//* ================================================================

//$ Function Check Switch A is Pressed
u8 is_SwA(void) { return (PINB & (1 << PINB0)); }

//+ wait for Switch A - Press
void wait_SwA(void)
{
	while (!is_SwA())
		;
}

//+ wait for Switch A - unPress
void wait_SwA_unPress(void)
{
	while (is_SwA())
		;
}

//* ================================================================
//. LED A
//* ================================================================

//$ LED A Control - ON
void LED_A__ON(void)
{
	//- Set Pin LED_A as Output
	/// LED A : PB0 -> output
	DDRB |= (1 << DDB0);

	//- Set Pin LED_A - HIGH
	/// LED A : PB0 => HIGH
	PORTB |= (1 << PORTB0);
}

//$ LED A Control - OFF
void LED_A__OFF(void)
{
	//- Set Pin LED_A - LOW
	/// LED A : PB0 => LOW
	PORTB &= ~((1 << PORTB0));

	//- Set Pin LED_A as Input
	/// LED A : PB0 -> input
	DDRB &= ~((1 << DDB0));
}

//+ blink LED_A
void blink_A(u16 duration_ms = 100, u8 times = 1)
{
	while (times--)
	{
		LED_A__ON();
		for (u16 i = 0; i < duration_ms; i++)
			_delay_ms(1);

		LED_A__OFF();
		for (u16 i = 0; i < duration_ms; i++)
			_delay_ms(1);
	}
}

//* ================================================================
//. LED B
//* ================================================================

//$ LED B Control - ON
void LED_B__ON(void)
{
	//- Set Pin LED_B as Output
	/// LED B : PD3 -> output
	DDRD |= (1 << DDD3);

	//- Set Pin LED_B - HIGH
	/// LED B : PD3 => HIGH
	PORTD |= (1 << PORTD3);
}

//$ LED B Control - OFF
void LED_B__OFF(void)
{
	//- Set Pin LED_B - LOW
	/// LED B : PD3 => LOW
	PORTD &= ~((1 << PORTD3));

	//- Set Pin LED_B as Input
	/// LED B : PD3 -> input
	DDRD &= ~((1 << DDD3));
}

//+ blink LED_B
void blink_B(u16 duration_ms = 100, u8 times = 1)
{
	while (times--)
	{
		LED_B__ON();
		for (u16 i = 0; i < duration_ms; i++)
			_delay_ms(1);

		LED_B__OFF();
		for (u16 i = 0; i < duration_ms; i++)
			_delay_ms(1);
	}
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void _____TINY_init_(void)
{
	_init_Motor();
	_init_SwitchAndLED();

#ifdef ___DISABLE_TIMER0_INTERRUPT___
	_init_Timer();
#endif

#ifdef ___ENABLE_SPI0___
	_init_SPI0();
#endif
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initVariant(void)
{
	_____TINY_init_();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~