/**
  ******************************************************************************
  * @file    main.c
  * @author  Jhony Benites, Brno University of Technology, Czechia
	     Originally from labs guided by doc. Ing. Tomas Fryza, Ph.D.
  * @version V1.1
  * @date    Nov 19, 2018
  * @brief   Home Security System with 2 PIR sensors, LCD interface and buzzer
  * @note    Project developed on Atmel Studio 7.0 and built with GCC.
  *          Modified version of Peter Fleury's LCD library is used.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include "lcd.h"

/* Constants and macros ------------------------------------------------------*/
/**
 *  @brief LCD text blink time in ms.
 */
#define LCD_TEXT_BLINK_TIME 600
/**
 *  @brief Time for button debouncing in ms via shift register.
 */
#define BUTTON_DEBOUNCE_TIME 5
/**
 *  @brief Sample period in ms for PIR sensors.
 */
#define PIR_SENSOR_SAMPLE_PERIOD 100
/**
 *  @brief Timeout in ms for entering PIN (PUK) code.
 */
#define CODE_TIMEOUT 30000
/**
 *  @brief ADC threshold for buttons handling.
 */
#define RIGHT_BUTTON_THRESHOLD 75
#define UP_BUTTON_THRESHOLD 230
#define DOWN_BUTTON_THRESHOLD 360
#define LEFT_BUTTON_THRESHOLD 625
#define SELECT_BUTTON_THRESHOLD 850

/* Global variables ----------------------------------------------------------*/
/**
 *  @brief Pointer to EEPROM for PUK code.
 */
static uint8_t *PUK = (uint8_t*)4;
/**
 *  @brief Pointer to EEPROM for PIN code.
 */
static uint8_t *PIN = (uint8_t*)0;

/* Structures ----------------------------------------------------------*/
/**
 *  @brief Flags for pressed buttons.
 */
struct
{
	unsigned int left   : 1;
	unsigned int right  : 1;
	unsigned int up     : 1;
    unsigned int down       : 1;
	unsigned int select : 1;
} buttons_pressed;

/**
 *  @brief Flags for debounced buttons.
 */
struct
{
	unsigned int left   : 1;
	unsigned int right  : 1;
	unsigned int up     : 1;
	unsigned int down   : 1;
	unsigned int select : 1;
} buttons_debounced;

/**
 *  @brief General flags.
 */
struct
{
	unsigned int LCD_text_blink	: 1;
	unsigned int button_debounce	: 1;
	unsigned int code_timeout	: 1;
	unsigned int PIR_sensor_sample	: 1;
	unsigned int alarm_both         : 1;
	unsigned int alarm_left         : 1;
	unsigned int alarm_right        : 1;
	unsigned int alarm		: 1;
	unsigned int pin_mode		: 1;
	unsigned int puk_mode		: 1;
	unsigned int change_pin_mode	: 1;
	unsigned int new_pin_mode       : 1;
} flags;

/**
 *  @brief Flags for enabling timers.
 */
struct
{
	unsigned int LCD_text_blink     : 1;
	unsigned int button_debounce    : 1;
	unsigned int code_timeout       : 1;
	unsigned int PIR_sensor_sample  : 1;
} timers_enable;

/* Function prototypes -------------------------------------------------------*/
void board_init(void);
void systick_1ms(void);
void button_debounce(void);
void LCD_text_blink(void);
void PIR_sensor_sample(void);
void buttons_service(void);
void code_timeout(void);

/* Functions -----------------------------------------------------------------*/
/**
  * @brief Main function.
  */
int main(void)
{
	board_init();
	
	/* Cycle for initial PIN and PUK code */
	/* @note Cycle will remain commented in order to be able to store new
	   changed PIN into EEPROM.
	/*for(uint8_t i = 0; i < 4; i++)
	{
		eeprom_busy_wait();
		eeprom_write_byte(&PIN[i], 0);
	}
	
	for(uint8_t i = 0; i < 8; i++)
	{
		eeprom_busy_wait();
		eeprom_write_byte(&PUK[i], i);
	}*/
	
	while(1)
	{
		if(flags.LCD_text_blink)
		{
			LCD_text_blink();
			flags.LCD_text_blink = 0;
		}
		
		if(flags.button_debounce)
		{
			button_debounce();
			flags.button_debounce = 0;
		}
		
		if(flags.PIR_sensor_sample)
		{
			PIR_sensor_sample();
			flags.PIR_sensor_sample = 0;
		}
		
		if(flags.code_timeout)
		{
			code_timeout();
			flags.code_timeout = 0;
		}
	}
			
	return (0);
}

/**
  * @brief Initializes board target resources to be used.
  */
void board_init(void)
{
	buttons_debounced.down = 0;
	buttons_debounced.up = 0;
    	buttons_debounced.left = 0;
	buttons_debounced.right = 0;
	buttons_debounced.select = 0;

	buttons_pressed.down = 0;
	buttons_pressed.up = 0;
	buttons_pressed.left = 0;
	buttons_pressed.right = 0;
	buttons_pressed.select = 0;

	flags.LCD_text_blink = 0;
	flags.button_debounce = 0;
	flags.code_timeout = 0;
	flags.PIR_sensor_sample = 0;
	flags.alarm = 0;
	flags.alarm_both = 0;
	flags.alarm_left = 0;
	flags.alarm_right = 0;
	flags.pin_mode = 0;
	flags.puk_mode = 0;
	flags.change_pin_mode = 0;
	flags.new_pin_mode = 0;
	
	timers_enable.button_debounce = 1;
	timers_enable.LCD_text_blink = 0;
	timers_enable.code_timeout = 0;
	timers_enable.PIR_sensor_sample = 1;
	
	/* Initialize display with cursor off */
	lcd_init(LCD_DISP_ON);
	
	/* Clear display and set cursor to home position */
	lcd_clrscr();
	
	/* Set left sensor as input */
	DDRB &= ~(1 << PORTB3);
	
	/* Set right sensor as input */
	DDRB &= ~(1 << PORTB4);
	
	/* Set buzzer as output */
	DDRB |= (1 << PORTB5);
	
	/* Turn buzzer off */
	PORTB &= ~(1 << PORTB5);
	
	/* Set cursor to specified position */
	lcd_gotoxy(4,0);
	
	/* Display string without auto linefeed */
	lcd_puts("HOME IS");
	lcd_gotoxy(4,1);
	lcd_puts("SAFE :)");

	/*ADC for buttons init */
	/*ADC0 channel, voltage reference to AVCC */
	ADMUX |= (1 << REFS0);
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
	
	/* Timer/Counter0: Select clock and enable compare match */
	/* Clear timer on compare match */
	TCCR0A |= (1 << WGM01);
	
	/* Clock prescaler 64 => clock prescaled to 250 kHz */
    	TCCR0B |= (1 << CS01) | (1 << CS00);
	
	/* Output Compare register A set to 250 => interrupt every 1ms */
	OCR0A = 250;
	
	/* Compare match A interrupt enable */
	TIMSK0 |= (1 << OCIE0A);

    	/*Enable global interrupts*/
	sei();
	
	return;
}	

/**
  * @brief Function for buttons debouncing.
  */
void button_debounce(void)
{
	static volatile uint16_t button_up_debounce = 0xffff;
	static volatile uint16_t button_down_debounce = 0xffff;
	static volatile uint16_t button_left_debounce = 0xffff;
	static volatile uint16_t button_right_debounce = 0xffff;
	static volatile uint16_t button_select_debounce = 0xffff;
	
	/* Start single AD conversion */
	ADCSRA |= (1 << ADSC);
	
	button_up_debounce <<= 1;
	button_down_debounce <<= 1;
	button_left_debounce <<= 1;
	button_right_debounce <<= 1;
	button_select_debounce <<= 1;

	button_up_debounce |= (!!buttons_pressed.up);
	button_down_debounce |= (!!buttons_pressed.down);
	button_left_debounce |= (!!buttons_pressed.left);
	button_right_debounce |= (!!buttons_pressed.right);
	button_select_debounce |= (!!buttons_pressed.select);

	if(button_up_debounce == 0x000f)
	{
		buttons_debounced.up = 1;
		buttons_service();
	}
	if(button_down_debounce == 0x000f)
	{
		buttons_debounced.down = 1;
		buttons_service();
	}
	if(button_left_debounce == 0x000f)
	{
		buttons_debounced.left = 1;
		buttons_service();
	}
	if(button_right_debounce == 0x000f)
	{
		buttons_debounced.right = 1;
		buttons_service();
	}
	if(button_select_debounce == 0x000f)
	{
		buttons_debounced.select = 1;
		buttons_service();
	}
	
	return;
}

/**
  * @brief Function for blinking text in LCD.
  */
void LCD_text_blink(void)
{
	static uint8_t state = 0;

	if(state)
	{
		state = 0;
		lcd_command(LCD_DISP_OFF);
	}
	else
	{
		state = 1;
		lcd_command(LCD_DISP_ON);
	}

	return;
}

/**
  * @brief Function for sampling of PIR sensor.
  */
void PIR_sensor_sample(void)
{
	static uint8_t state_old_left_sensor = 0;
	static uint8_t state_old_right_sensor = 0;
	uint8_t state_new_left_sensor = 0, state_new_right_sensor = 0;

	state_new_right_sensor = PINB & (1 << PORTB4);
    	state_new_left_sensor = PINB & (1 << PORTB3);

	/*  Detection of rising edge */
	if(state_new_left_sensor > state_old_left_sensor && state_new_right_sensor > state_old_right_sensor && (!flags.alarm_both))
	{
		PORTB |= (1 << PORTB5);
		lcd_clrscr();
		lcd_gotoxy(4,0);
		lcd_puts("WARNING!");
		lcd_gotoxy(2,1);
		lcd_puts("BOTH SENSORS");
		flags.alarm = 1;
		flags.alarm_both = 1;
		timers_enable.LCD_text_blink = 1;
	}
	
	/* Detection of rising edge for left sensor */
	else if(state_new_left_sensor > state_old_left_sensor && (!flags.alarm_both) && (!flags.alarm_left)) 
	{
		if(flags.alarm_right)
		{
			PORTB |= (1 << PORTB5);
			lcd_clrscr();
			lcd_gotoxy(4,0);
			lcd_puts("WARNING!");
			lcd_gotoxy(2,1);
			lcd_puts("BOTH SENSORS");
			flags.alarm = 1;
			flags.alarm_both = 1;
		}
		else
		{
			PORTB |= (1 << PORTB5);
			lcd_clrscr();
			lcd_gotoxy(4,0);
			lcd_puts("WARNING!");
			lcd_gotoxy(2,1);
			lcd_puts("LEFT SENSOR");
			flags.alarm = 1;
			flags.alarm_left = 1;
		}
		timers_enable.LCD_text_blink = 1;
	}
	
	/* Detection of rising edge for right sensor */
	else if(state_new_right_sensor > state_old_right_sensor && (!flags.alarm_both) && (!flags.alarm_right))
	{
		if(flags.alarm_left)
		{
			PORTB |= (1 << PORTB5);
			lcd_clrscr();
			lcd_gotoxy(4,0);
			lcd_puts("WARNING!");
			lcd_gotoxy(2,1);
			lcd_puts("BOTH SENSORS");
			flags.alarm = 1;
			flags.alarm_both = 1;
		}
		else
		{
			PORTB |= (1 << PORTB5);
			lcd_clrscr();
			lcd_gotoxy(4,0);
			lcd_puts("WARNING!");
			lcd_gotoxy(2,1);
			lcd_puts("RIGHT SENSOR");
			flags.alarm = 1;
			flags.alarm_right = 1;
		}
		timers_enable.LCD_text_blink = 1;
	}

	state_old_right_sensor = state_new_right_sensor;
	state_old_left_sensor = state_new_left_sensor;
}

/**
  * @brief Performs tasks according to pressed buttons and set flags.
  */
void buttons_service(void)
{
	static int8_t pin_input[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
	static uint8_t cursor_position = 0;
	static uint8_t code_length = 4;
	static uint32_t number_of_attempts = 0;
	uint8_t pin_correct = 0;
	char c[2];

	if(buttons_debounced.select)
	{
		if((flags.alarm && (!flags.pin_mode)) || ((!flags.alarm) && (!flags.change_pin_mode) && (!flags.new_pin_mode)))
		{
			timers_enable.LCD_text_blink = 0;
			if(flags.alarm)
				flags.pin_mode = 1;
			else
				flags.change_pin_mode = 1;
			pin_input[0] = -1; pin_input[1] = -1; pin_input[2] = -1; pin_input[3] = -1;
			pin_input[4] = -1; pin_input[5] = -1; pin_input[6] = -1; pin_input[7] = -1;
			cursor_position = 0;
			lcd_clrscr();
			lcd_command(LCD_DISP_ON);
			lcd_command(LCD_DISP_ON_CURSOR_BLINK);
			lcd_gotoxy(0,0);
			if(number_of_attempts < 3)
				lcd_puts("ENTER PIN");
			else
				lcd_puts("ENTER PUK");
			lcd_gotoxy(0,1);
			timers_enable.code_timeout = 1;	
		}
		
		else if(flags.pin_mode || flags.change_pin_mode)
		{
			if((pin_input[0] > -1 && pin_input[1] > -1 && pin_input[2] > -1 && pin_input[3] > -1 && number_of_attempts < 3) ||
			   (pin_input[0] > -1 && pin_input[1] > -1 && pin_input[2] > -1 && pin_input[3] > -1 && pin_input[4] > -1 && pin_input[5] > -1 && pin_input[6] > -1 && pin_input[7] > -1))
			{
				pin_correct = 1;
				
				if(number_of_attempts < 3)
					for(uint8_t i = 0; i < 4; i++)
					{
						eeprom_busy_wait();
						if(pin_input[i] != eeprom_read_byte(&PIN[i]))
							pin_correct = 0;
					}
				else
					for(uint8_t i = 0; i < 8; i++)
					{
						eeprom_busy_wait();
						if(pin_input[i] != eeprom_read_byte(&PUK[i]))
						pin_correct = 0;
					}
				
				if(pin_correct)
				{
					flags.alarm = 0;
					flags.alarm_both = 0;
					flags.alarm_right = 0;
					flags.alarm_left = 0;
					timers_enable.LCD_text_blink = 0;
					PORTB &= ~ (1 << PORTB5);
					lcd_command(LCD_DISP_ON);
					if(flags.pin_mode)
					{
						lcd_clrscr();
						lcd_gotoxy(4,0);
						lcd_puts("HOME IS");
						lcd_gotoxy(4,1);
						lcd_puts("SAFE :)");
						timers_enable.code_timeout = 0;
					}
					else if(flags.change_pin_mode)
					{
						lcd_clrscr();
						lcd_gotoxy(0,0);
						lcd_puts("ENTER NEW PIN");
						lcd_command(LCD_DISP_ON_CURSOR_BLINK);
						lcd_gotoxy(0,1);
						flags.new_pin_mode = 1;
						cursor_position = 0;
						pin_input[0] = -1; pin_input[1] = -1; pin_input[2] = -1; pin_input[3] = -1;
						pin_input[4] = -1; pin_input[5] = -1; pin_input[6] = -1; pin_input[7] = -1;
						timers_enable.code_timeout = 1;
					}
						
					number_of_attempts = 0;
					code_length = 4;
				}
				else
				{
					if(flags.alarm)
					{
						timers_enable.LCD_text_blink = 1;
						lcd_command(LCD_DISP_ON);
						lcd_clrscr();
						lcd_gotoxy(4,0);
						lcd_puts("WARNING!");
						lcd_gotoxy(2,1);
						if(flags.alarm_both)
							lcd_puts("BOTH SENSORS");
						else if(flags.alarm_left)
							lcd_puts("LEFT SENSOR");
						else
							lcd_puts("RIGHT SENSOR");
						timers_enable.code_timeout = 0;
					}
					else if(flags.change_pin_mode)
					{
						timers_enable.LCD_text_blink = 0;
						lcd_command(LCD_DISP_ON);
						lcd_clrscr();
						lcd_gotoxy(4,0);
						lcd_puts("HOME IS");
						lcd_gotoxy(4,1);
						lcd_puts("SAFE :)");
						timers_enable.code_timeout = 0;
					}
					number_of_attempts++;
					if(number_of_attempts >= 3)
						code_length = 8;
				}
			}
			else
			{
				if(flags.alarm)
				{
					timers_enable.LCD_text_blink = 1;
					lcd_command(LCD_DISP_ON);
					lcd_clrscr();
					lcd_gotoxy(4,0);
					lcd_puts("WARNING!");
					lcd_gotoxy(2,1);
					if(flags.alarm_both)
					lcd_puts("BOTH SENSORS");
					else if(flags.alarm_left)
					lcd_puts("LEFT SENSOR");
					else
					lcd_puts("RIGHT SENSOR");
					timers_enable.code_timeout = 0;
				}
				else if(flags.change_pin_mode)
				{
					timers_enable.LCD_text_blink = 0;
					lcd_command(LCD_DISP_ON);
					lcd_clrscr();
					lcd_gotoxy(4,0);
					lcd_puts("HOME IS");
					lcd_gotoxy(4,1);
					lcd_puts("SAFE :)");
					timers_enable.code_timeout = 0;
				}
				number_of_attempts++;
				if(number_of_attempts >= 3)
				code_length = 8;
			}
			flags.pin_mode = 0;
			flags.change_pin_mode = 0;
		}
		else if(flags.new_pin_mode)
		{
			for(uint8_t i = 0; i < 4; i++)
			{
				eeprom_busy_wait();
				eeprom_write_byte(&PIN[i], pin_input[i]);
			}
			
			lcd_clrscr();
			lcd_command(LCD_DISP_ON);
			lcd_gotoxy(4,0);
			lcd_puts("HOME IS");
			lcd_gotoxy(4,1);
			lcd_puts("SAFE :)");
			timers_enable.code_timeout = 0;
			
			flags.new_pin_mode = 0;
		}

		buttons_debounced.select = 0;
	}
	else if(buttons_debounced.up)
	{
		if(flags.pin_mode || flags.puk_mode || flags.change_pin_mode || flags.new_pin_mode)
		{
			if(pin_input[cursor_position] < 9)
				pin_input[cursor_position]++;
			else
				pin_input[cursor_position] = 0;
			sprintf(c, "%d", pin_input[cursor_position]);
			lcd_gotoxy(cursor_position,1);
			lcd_putc(c[0]);
			lcd_gotoxy(cursor_position,1);
		}

		buttons_debounced.up = 0;		
	}
	else if(buttons_debounced.down)
	{
		if(flags.pin_mode || flags.puk_mode || flags.change_pin_mode || flags.new_pin_mode)
		{
			if(pin_input[cursor_position] > 0)
				pin_input[cursor_position]--;
			else
				pin_input[cursor_position] = 9;
			sprintf(c, "%d", pin_input[cursor_position]);
			lcd_gotoxy(cursor_position,1);
			lcd_putc(c[0]);
			lcd_gotoxy(cursor_position,1);
		}

		buttons_debounced.down = 0;
	}
	else if(buttons_debounced.right)
	{
		if(flags.pin_mode || flags.puk_mode || flags.change_pin_mode || flags.new_pin_mode)
		{
			if(cursor_position < code_length - 1)
			{
				cursor_position++;
				lcd_gotoxy(cursor_position,1);
			}
		}

		buttons_debounced.right = 0;
	}
	else if(buttons_debounced.left)
	{
		if(flags.pin_mode || flags.puk_mode || flags.change_pin_mode || flags.new_pin_mode)
		{
			if(cursor_position > 0)
			{
				cursor_position--;
				lcd_gotoxy(cursor_position,1);
			}
		}

		buttons_debounced.left = 0;
	}

	return;
}

/**
  * @brief Function for timeout while entering PIN (PUK).
  */
void code_timeout(void)
{
	timers_enable.code_timeout = 0;
	if(flags.alarm)
	{
		timers_enable.LCD_text_blink = 1;
		lcd_command(LCD_DISP_ON);
		lcd_clrscr();
		lcd_gotoxy(4,0);
		lcd_puts("WARNING!");
		lcd_gotoxy(2,1);
		if(flags.alarm_both)
		lcd_puts("BOTH SENSORS");
		else if(flags.alarm_left)
		lcd_puts("LEFT SENSOR");
		else
		lcd_puts("RIGHT SENSOR");
	}
	else
	{
		timers_enable.LCD_text_blink = 0;
		lcd_clrscr();
		lcd_command(LCD_DISP_ON);
		lcd_gotoxy(4,0);
		lcd_puts("HOME IS");
		lcd_gotoxy(4,1);
		lcd_puts("SAFE :)");
	}
	flags.change_pin_mode = 0;
	flags.code_timeout = 0;
	flags.new_pin_mode = 0;
	flags.pin_mode = 0;
	flags.puk_mode = 0;
	
	return;
}

/**
  * @brief Generates periodically interrupt requests.
  */
void systick_1ms(void)
{
	static volatile uint16_t LCD_blink_timer_1ms = 0;
	static volatile uint8_t debounce_timer_1ms = 0;
	static volatile uint16_t code_timeout_timer_1ms = 0;
	static volatile uint16_t PIR_sensor_sample_timer_1ms = 0;
	
	if(timers_enable.LCD_text_blink)
	{
	    LCD_blink_timer_1ms++;
		if(LCD_blink_timer_1ms == LCD_TEXT_BLINK_TIME)
		{
			LCD_blink_timer_1ms = 0;
			flags.LCD_text_blink = 1;
		}
	}
	else
	    LCD_blink_timer_1ms = 0;
	
	if(timers_enable.button_debounce)
	{
	    debounce_timer_1ms++;
		if(debounce_timer_1ms == BUTTON_DEBOUNCE_TIME)
		{
			debounce_timer_1ms = 0;
			flags.button_debounce = 1;
		}
	}
	else
		debounce_timer_1ms = 0;
	
	if(timers_enable.code_timeout)
	{
		code_timeout_timer_1ms++;
		if(code_timeout_timer_1ms == CODE_TIMEOUT)
		{
			code_timeout_timer_1ms = 0;
			flags.code_timeout = 1;
		}
	}
	else
	code_timeout_timer_1ms = 0;
	
	if(timers_enable.PIR_sensor_sample)
	{
		PIR_sensor_sample_timer_1ms++;
		if(PIR_sensor_sample_timer_1ms == PIR_SENSOR_SAMPLE_PERIOD)
		{
			PIR_sensor_sample_timer_1ms = 0;
			flags.PIR_sensor_sample = 1;
		}
	}
	else
	PIR_sensor_sample_timer_1ms = 0;
	
	return;
}

/**
  * @brief Automatic start of a new Analog to Digital conversion.
  */
ISR(TIMER0_COMPA_vect)
{
	systick_1ms();
	
	return;
}

/**
  * @brief Analog to Digital conversion was completed.
  */
ISR(ADC_vect)
{
	volatile uint16_t ADC_value = 0;

	/* Read 10-bit value from ADC */
	ADC_value = ADC;

	buttons_pressed.down = 0;
	buttons_pressed.up = 0;
	buttons_pressed.left = 0;
	buttons_pressed.right = 0;
	buttons_pressed.select = 0;
	
	if(ADC_value < RIGHT_BUTTON_THRESHOLD)
		buttons_pressed.right = 1;
	else if(ADC_value < UP_BUTTON_THRESHOLD)
		buttons_pressed.up = 1;
	else if(ADC_value < DOWN_BUTTON_THRESHOLD)
		buttons_pressed.down = 1;
	else if(ADC_value < LEFT_BUTTON_THRESHOLD)
		buttons_pressed.left = 1;
	else if(ADC_value < SELECT_BUTTON_THRESHOLD)
		buttons_pressed.select = 1;

	return;
}

/* END OF FILE ****************************************************************/

