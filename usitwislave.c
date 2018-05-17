/*	See LICENSE for Copyright etc. */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "usitwislave.h"

enum
{
	of_state_check_address,
	of_state_send_data,
	of_state_request_ack,
	of_state_check_ack,
	of_state_receive_data,
	of_state_store_data_and_send_ack
} overflow_state_t;

enum
{
	ss_state_before_start,
	ss_state_after_start,
	ss_state_address_selected,
	ss_state_address_not_selected,
	ss_state_data_processed
} startstop_state_t;

static void (*data_callback)(uint8_t input_buffer_length,
			     const uint8_t *input_buffer,
			     uint8_t *output_buffer_length,
			     uint8_t *output_buffer);

static uint8_t of_state;
static uint8_t ss_state;

static uint8_t	slave_address;

static uint8_t	input_buffer[USI_TWI_BUFFER_SIZE];
static uint8_t	input_buffer_length;
static uint8_t	output_buffer[USI_TWI_BUFFER_SIZE];
static uint8_t	output_buffer_length;
static uint8_t	output_buffer_current;

static void set_sda_to_input(void)
{
	DDR_USI &= ~_BV(PORT_USI_SDA);
}

static void set_sda_to_output(void)
{
	DDR_USI |= _BV(PORT_USI_SDA);
}

static inline void set_scl_to_input(void)
{
	DDR_USI &= ~_BV(PORT_USI_SCL);
}

static inline void set_scl_to_output(void)
{
	DDR_USI |= _BV(PORT_USI_SCL);
}

static inline void set_sda_low(void)
{
	PORT_USI &= ~_BV(PORT_USI_SDA);
}

static inline void set_sda_high(void)
{
	PORT_USI |= _BV(PORT_USI_SDA);
}

static inline void set_scl_low(void)
{
	PORT_USI &= ~_BV(PORT_USI_SCL);
}

static inline void set_scl_high(void)
{
	PORT_USI |= _BV(PORT_USI_SCL);
}

static inline void twi_reset_state(void)
{
	USISR =
		/* clear start condition flag */
		(1 << USISIF) |
		/* clear overflow condition flag */
		(1 << USIOIF) |
		/* !clear stop condition flag */
		(0 << USIPF) |
		/* clear arbitration error flag	*/
		(1 << USIDC) |
		/* set counter to "8" bits */
		(0x00	<< USICNT0);

	USICR =
		/* enable start condition interrupt */
		(1 << USISIE) |
		/* !enable overflow interrupt */
		(0 << USIOIE) |
		/* set usi in two-wire mode */
		(1 << USIWM1) |
		/* disable bit counter overflow hold */
		(0 << USIWM0) |
		/*
		  shift register clock source = external, positive
		  edge, 4-bit counter source = external, both edges
		*/
		(1 << USICS1) |	(0 << USICS0) |	(0 << USICLK) |
		/* don't toggle clock-port pin */
		(0 << USITC);
}

static void twi_reset(void)
{
	/* make sure no sda/scl remains pulled up or down */

	set_sda_to_input(); /* deactivate internal pullup on sda/scl */
	set_sda_low();
	set_scl_to_input();
	set_scl_low();

	set_sda_to_output(); /* release (set high) on sda/scl */
	set_sda_high();
	set_sda_to_input();
	set_sda_high();
	set_scl_to_output();
	set_scl_high();

	twi_reset_state();
}

static inline void twi_init(void)
{
#if defined(USIPP)
#if  defined(USI_ON_PORT_A)
	USIPP |= _BV(USIPOS);
#else
	USIPP &= ~_BV(USIPOS);
# endif
#endif

	twi_reset();
}

ISR(USI_START_vect)
{
	set_sda_to_input();

	/*
	  Wait for SCL to go low to ensure that the start condition
	  has completed (the start detector will hold SCL low) - if a
	  stop condition arises then leave the interrupt to prevent
	  waiting forever - don't use USISR to test for stop condition
	  as in Application Note AVR312 because the stop condition
	  Flag is going to be set from the last TWI sequence
	*/

	while (!(PIN_USI & _BV(PIN_USI_SDA)) && (PIN_USI & _BV(PIN_USI_SCL)));

	/*
	  -- possible combinations --

	  sda = low    scl = low    break    start condition
	  sda = low    scl = high   loop
	  sda = high   scl = low    break    stop condition
	  sda = high   scl = high   break    stop condition
	*/

	if ((PIN_USI & _BV(PIN_USI_SDA))) { /* stop condition */
		twi_reset();

		return;
	}

	of_state = of_state_check_address;
	ss_state = ss_state_after_start;

	USIDR = 0xff;

	USICR =
		/*
		  enable start condition interrupt
		*/
		(1 << USISIE) |
		/*
		  enable overflow interrupt
		*/
		(1 << USIOIE) |
		/*
		  set usi in two-wire mode, enable bit counter
		  overflow hold
		*/
		(1 << USIWM1) | (1 << USIWM0) |
		/*
		  shift register clock source = external, positive
		  edge, 4-bit counter source = external, both edges
		*/
		(1 << USICS1) | (0 << USICS0) | (0 << USICLK) |
		/*
		  don't toggle clock-port pin
		*/
		(0 << USITC);

	USISR =
		/* clear start condition flag */
		(1 << USISIF) |
		/* clear overflow condition flag */
		(1 << USIOIF) |
		/* !clear stop condition flag */
		(0 << USIPF) |
		/* clear arbitration error flag	*/
		(1 << USIDC) |
		/* set counter to "8" bits */
		(0x00	<< USICNT0);
}

ISR(USI_OVERFLOW_VECTOR)
{
	/*
	  bit shift register overflow condition occured scl forced low
	  until overflow condition is cleared!
	*/

	uint8_t data = USIDR;
	uint8_t set_counter = 0x00; /* send 8 bits (16 edges) */

again:

	switch (of_state) {

	/*
	  start condition occured and succeed check address,
	  if not OK, reset usi note: not using general call
	  address
	*/

	case (of_state_check_address):
	{
		uint8_t address;
		uint8_t direction;

		direction = data & 0x01;
		address	= (data & 0xfe) >> 1;

		if (address == slave_address) {
			ss_state = ss_state_address_selected;

			if(direction)
				/* read request from master */
				of_state = of_state_send_data;
			else
				/* write request from master */
				of_state = of_state_receive_data;

			USIDR = 0x00;
			set_counter = 0x0e; /* send 1 bit (2 edges) */
			set_sda_to_output(); /* initiate send ack */
		} else {
			USIDR = 0x00;
			set_counter = 0x00;
			twi_reset_state();
			ss_state = ss_state_address_not_selected;
		}

		break;
	}

	/*
	  process read request from master
	*/

	case (of_state_send_data):
		ss_state = ss_state_data_processed;
		of_state = of_state_request_ack;

		if (output_buffer_current < output_buffer_length)
			USIDR = output_buffer[output_buffer_current++];
		else
			/* no more data, but cannot send "nothing" or "nak" */
			USIDR = 0x00;

		set_counter = 0x00;
		set_sda_to_output(); /* initiate send data */
		break;

	/*
	  data sent to master, request ack (or nack) from master
	*/

	case (of_state_request_ack):
		of_state = of_state_check_ack;
		USIDR = 0x00;
		set_counter = 0x0e; /* receive 1 bit (2 edges) */
		set_sda_to_input(); /* initiate receive ack */
		break;

	/*
	  ack/nack from master received
	*/

	case (of_state_check_ack):
		if (data) {
			/* if NACK, the master does not want more data */
			of_state = of_state_check_address;
			set_counter = 0x00;
			twi_reset();
		} else {
			of_state = of_state_send_data;
			/* drop straight into state_send_data */
			goto again;
		}

		/* don't wait for another overflow interrupt */
		break;
		
	/*
	  process write request from master
	*/

	case (of_state_receive_data):
		ss_state = ss_state_data_processed;
		of_state = of_state_store_data_and_send_ack;
		set_counter = 0x00; /* receive 1 bit (2 edges) */
		set_sda_to_input(); /* initiate receive data */
		break;

	/*
	  data received from master, store it and wait for more data
	*/

	case (of_state_store_data_and_send_ack):
		of_state = of_state_receive_data;

		if(input_buffer_length < (USI_TWI_BUFFER_SIZE - 1))
			input_buffer[input_buffer_length++] = data;

		USIDR		= 0x00;
		set_counter = 0x0e; /* send 1 bit (2 edges) */
		set_sda_to_output(); /* initiate send ack */
		break;
	}

	USISR =
		/* don't clear start condition flag */
		(0 << USISIF) |
		/* clear overflow condition flag */
		(1 << USIOIF) |
		/* don't clear stop condition flag */
		(0 << USIPF) |
		/* clear arbitration error flag */
		(1 << USIDC) |
		/* set counter to 8 or 1 bits */
		(set_counter << USICNT0);
}

void
usi_twi_slave(uint8_t slave_address_in,
	      void (*data_callback_in)(uint8_t input_buffer_length,
				       const uint8_t *input_buffer,
				       uint8_t *output_buffer_length,
				       uint8_t *output_buffer))
{
	slave_address = slave_address_in;
	data_callback = data_callback_in;

	input_buffer_length = 0;
	output_buffer_length = 0;
	output_buffer_current = 0;
	ss_state = ss_state_before_start;

	twi_init();
	sei();

	return;
}

void
usi_twi_check(void)
{
	int call_datacallback = 0;

	if(USISR & _BV(USIPF)) {
		cli();

		USISR |= _BV(USIPF); /* clear stop condition flag */

		switch(ss_state) {
		case (ss_state_after_start):
			twi_reset();
			break;
		case (ss_state_data_processed):
			call_datacallback = 1;
			break;
		}

		ss_state = ss_state_before_start;

		sei();
	}

	if (call_datacallback) {
		output_buffer_length = 0;
		output_buffer_current = 0;
		data_callback(input_buffer_length, input_buffer,
			      &output_buffer_length, output_buffer);
		input_buffer_length = 0;
		call_datacallback = 0;
	}

	return;
}
