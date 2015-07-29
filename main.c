/*
 * AVR-Firmware for a sensor board for LEGO NXT robots.
 * Created by Max (max.electronics-at-me.com),
 * using some I2C code by Donald R. Blake that was somehow modified by Moritz Strübe
 * 
 * License: GPL 2.0
 *
 * This only refers to the 2nd Version, using 5 I2C-color-sensors and an ATtiny261A (probably...)
 * For the 1st Version, please refer to Andi...
 *
 * This code is self-documenting (or I'm just too lazy to write comments)
 */ 
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USI_ADDRESS 24

static uint8_t write = 0;
static uint8_t buffer[2][40];
static volatile uint8_t buffer_in_use = 0;

static uint8_t device_ID = 0;
static uint8_t set_gain = 0;

static uint8_t adc_0 = 0, adc_1 = 0;

static inline void i2c_wait(void) {
	while(!(TIFR & (1<<OCF0A)));
	TIFR |= (1<<OCF0A);
}

static void sensor_start(uint8_t read) {
	i2c_wait();
	DDRA |= 0xDC;
	i2c_wait();
	DDRA |= (1<<PA1);
	
	uint8_t adress = 0x52 + (read&1);
	for(uint8_t i = 0; i < 8; i++) {
		if(adress&0x80) DDRA &= ~0xDC;
		else DDRA |= 0xDC;
		i2c_wait();
		DDRA &= ~(1<<PA1);
		i2c_wait();
		DDRA |= (1<<PA1);
		adress <<= 1;
	}
	DDRA &= ~0xDC;
	i2c_wait();
	DDRA &= ~(1<<PA1);
	i2c_wait();
	DDRA |= (1<<PA1);
}

static void sensor_write(uint8_t data) {
	for(uint8_t i = 0; i < 8; i++) {
		if(data&0x80) DDRA &= ~0xDC;
		else DDRA |= 0xDC;
		i2c_wait();
		DDRA &= ~(1<<PA1);
		i2c_wait();
		DDRA |= (1<<PA1);
		data <<= 1;
	}
	DDRA &= ~0xDC;
	i2c_wait();
	DDRA &= ~(1<<PA1);
	i2c_wait();
	DDRA |= (1<<PA1);
}

static void sensor_read(uint8_t *out, uint8_t ack) {
	DDRA &= ~0xDC;
	for(uint8_t i = 0; i < 8; i++) {
		out[0] <<= 1;
		out[1] <<= 1;
		out[2] <<= 1;
		out[3] <<= 1;
		out[4] <<= 1;
		i2c_wait();
		DDRA &= ~(1<<PA1);
		if(PINA&(1<<PA7)) out[0] |= 1;
		if(PINA&(1<<PA2)) out[1] |= 1;
		if(PINA&(1<<PA3)) out[2] |= 1;
		if(PINA&(1<<PA4)) out[3] |= 1;
		if(PINA&(1<<PA6)) out[4] |= 1;
		i2c_wait();
		DDRA |= (1<<PA1);
	}
	if(!ack)DDRA |= 0xDC;
	i2c_wait();
	DDRA &= ~(1<<PA1);
	i2c_wait();
	DDRA |= (1<<PA1);
	DDRA &= ~0xDC;
}

static void sensor_stop() {
	DDRA |= 0xDC;
	i2c_wait();
	DDRA &= ~(1<<PA1);
	i2c_wait();
	DDRA &= ~0xDC;
}

static uint8_t readADC(uint8_t channel) {
	uint16_t result = 0;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);
	ADMUX = (channel & 0x1F) | (1<<ADLAR);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	for (uint8_t i = 0; i < 4; i++) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		result += ADCH;
	}
	ADCSRA &= ~(1 << ADEN);
	result /= 4;
	return result;
}

static void TWI_init(void) {
	DDRB |= (1<<PB2) | (1<<PB0);
	PORTB |= (1<<PB2);
	PORTB |= (1<<PB0);
	DDRB &= ~(1<<PB0);

	USICR = (1<<USISIE) | (1<<USIWM1) | (1<<USICS1);
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
} 

int main(void) {
	OCR0A = 20;
	TCCR0A = (1<<0);
	TCCR0B = (1<<CS00);
	
	OCR1B = 0;
	OCR1D = 255;
	OCR1C = 255;
	TCCR1A = (1<<PWM1B);
	TCCR1B = (1<<CS10);
	TCCR1C = (1<<COM1B0S) | (1<<COM1D1) | (1<<COM1D0) | (1<<PWM1D);
	DDRB = (1<<PB3) | (1<<PB5);
	
	TWI_init();
	
	{
		sensor_start(0);
		sensor_write(0x92);
		sensor_stop();
		sensor_start(1);
		uint8_t IDs[5];
		sensor_read(IDs, 1);
		sensor_stop();
		device_ID = IDs[2];
	}
	
	sensor_start(0);
	sensor_write(0x80);
	sensor_write(0x01);
	sensor_stop();
	_delay_ms(10);
	sensor_start(0);
	sensor_write(0x80);
	sensor_write(0x03);
	sensor_stop();
	
	while(1) {
		if(set_gain) {
			sensor_start(0);
			sensor_write(0x8F);
			sensor_write(set_gain & 3);
			sensor_stop();
			set_gain = 0;
		}
		sensor_start(0);
		sensor_write(0xB4);
		sensor_stop();
		sensor_start(1);
		sensor_read(buffer[write]+ 0, 0);
		sensor_read(buffer[write]+ 5, 0);
		sensor_read(buffer[write]+10, 0);
		sensor_read(buffer[write]+15, 0);
		sensor_read(buffer[write]+20, 0);
		sensor_read(buffer[write]+25, 0);
		sensor_read(buffer[write]+30, 0);
		sensor_read(buffer[write]+35, 1);
		sensor_stop();
		
		adc_0 = readADC(0);
		adc_1 = readADC(4);
		
		cli();
		while(buffer_in_use) {
			sei();
			asm volatile("nop");
			cli();
		}
		write ^= 1;
		sei();
	}
}


static inline void SET_USI_TO_SEND_ACK() {
	USIDR = 0;
	DDRB |= (1<<PB0);
	USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0E<<USICNT0);
}

static inline void SET_USI_TO_READ_ACK() {
	DDRB &= ~(1<<PB0);
	USIDR = 0;
	USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0E<<USICNT0);
}

static inline void SET_USI_TO_TWI_START_CONDITION_MODE() {
	USICR = (1<<USISIE) | (1<<USIWM1) | (1<<USICS1);
	USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
}

static inline void SET_USI_TO_SEND_DATA() {
	DDRB |= (1<<PB0);
	USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
}

static inline void SET_USI_TO_READ_DATA() {
	DDRB &= ~(1<<PB0);
	USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
}

typedef enum {
	USI_SLAVE_CHECK_ADDRESS,
	USI_SLAVE_SEND_DATA,
	USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA,
	USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA,
	USI_SLAVE_REQUEST_DATA,
	USI_SLAVE_GET_DATA_AND_SEND_ACK
} overflowState_t;
static overflowState_t overflowState;
static uint8_t TWI_pos = 0;
static uint8_t TWI_command = 0;

ISR(USI_START_vect) {
	overflowState = USI_SLAVE_CHECK_ADDRESS;
	DDRB &= ~(1<<PB0);

	while ((PINB&(1<<PB2)) && !(PINB&(1<<PB0)));

	if (!(PINB&(1<<PB0))) {
		USICR = (1<<USISIE) | (1<<USIOIE) | (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);
	} else {
		TWI_pos = 0;
		USICR = (1<<USISIE) | (1<<USIWM1) | (1<<USICS1);
	}
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
}

ISR(USI_OVF_vect) {
	switch (overflowState) {

	case USI_SLAVE_CHECK_ADDRESS:
		if ((USIDR == 0) || ((USIDR >> 1) == USI_ADDRESS)) {
			if (USIDR & 0x01) {
				overflowState = USI_SLAVE_SEND_DATA;
				TWI_pos = 0;
			} else {
				overflowState = USI_SLAVE_REQUEST_DATA;
				TWI_pos = 0;
			}
			SET_USI_TO_SEND_ACK();
		} else {
			SET_USI_TO_TWI_START_CONDITION_MODE();
		}
		break;

	case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
		if (USIDR) {
			buffer_in_use = 0;
			SET_USI_TO_TWI_START_CONDITION_MODE();
			return;
		}
		/* no break */
		
	case USI_SLAVE_SEND_DATA:
		switch(TWI_command) {
		case 0x52:
			USIDR = device_ID;
			break;
		case 0x55:
		case 0x56:
		case 0x57:
		case 0x58:
		case 0x59:
			if(TWI_pos == 0) buffer_in_use = 1;
			if(TWI_pos < 8) {
				USIDR = buffer[write^1][TWI_pos*5+TWI_command-0x55];
			}
			if(TWI_pos == 7) buffer_in_use = 0;
			break;
		case 0x5A:
		case 0x5B:
		case 0x5C:
		case 0x5D:
			if(TWI_pos == 0) buffer_in_use = 1;
			if(TWI_pos < 10) {
				USIDR = buffer[write^1][TWI_pos+(TWI_command-0x5A)*10];
			}
			if(TWI_pos == 9) buffer_in_use = 0;
			break;
		case 0x5E:
			if(TWI_pos == 0) USIDR = adc_0;
			if(TWI_pos == 1) USIDR = adc_1;
			break;
		case 0x5F:
			if(TWI_pos == 0) USIDR = PINA & ((1<<PA5) | (1<<PA0));
			break;
		}
		
		if(TWI_pos < 255) TWI_pos++;
		
		overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
		SET_USI_TO_SEND_DATA();
		break;
		
	case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
		overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
		SET_USI_TO_READ_ACK();
		break;

	case USI_SLAVE_REQUEST_DATA:
		overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
		SET_USI_TO_READ_DATA();
		break;

	case USI_SLAVE_GET_DATA_AND_SEND_ACK:
		if(TWI_pos == 0) TWI_command = USIDR;
		switch(TWI_command) {
		case 0x53:
			if(TWI_pos == 1) OCR1D = 255-USIDR;
			else if(TWI_pos == 2) OCR1B = USIDR;
			break;
		case 0x54:
			if(TWI_pos == 1) set_gain = 0x80 | USIDR;
			break;
		}
		
		if(TWI_pos < 255) TWI_pos++;
		
		overflowState = USI_SLAVE_REQUEST_DATA;
		SET_USI_TO_SEND_ACK();
		break;
	}
}
