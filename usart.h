/********************************************************
* Description	: Library for send data through USART AVR
* Name			: usart.h
* Created		: Juan Lopez Medina
* Mail			: julome0@gmail.com
*********************************************************/

#define USART_BAUD	115200UL		// Define Baud rate

// Functions definitions
void usart_init(void);			// Initialize USART
int put_char(unsigned char);	// Tx character char
void put_string (char*);		// Tx string data
void put_int(int);				// Tx integer data
void put_long(long);			// Tx long data
void put_float(float);			// Tx float data
