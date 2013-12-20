/********************************************************
* Description	: Library for send data through USART AVR	
* Name			: usart.c
* Created		: Juan Lopez Medina
* Mail			: julome0@gmail.com
*********************************************************/

#define FCPU 16000000UL		// Define Clock AVR
#include "usart.h"
#include "avr/io.h"
#include "stdlib.h"

// Initialize USART
void usart_init(void){	
	UCSR0A |= (1 << U2X0);							// Config BAUDRATE
	UBRR0 = FCPU / (8 * USART_BAUD) - 1;	
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);			// Format 8N1 Asynchronous	
	UCSR0B = (1 << TXEN0);							// Enable module TX
}
// TX data char through USART
int put_char (unsigned char dato){
	while ((UCSR0A & (1 << UDRE0)) == 0);	// Wait for empty buffer
	UDR0 = dato;
	return dato;
}
// TX data string ASCII through USART
void put_string(char *s){
	while (*s){
		put_char(*s);
		s++;
	}
}
// TX integer variable through USART
void put_int (int dato){
	char s[7];
	itoa(dato,s,10);	// Converting data integer to ASCII
	put_string(s);
}
// TX long variable through USART
void put_long (long dato){
	char s[14];
	ltoa(dato,s,10);	// Converting data integer to ASCII
	put_string(s);
}
// Tx float variable through USART
void put_float (float dato){
	char s[15];
	dtostrf(dato,8,5,s);	// Converting data integer to ASCII
	put_string(s);
}



