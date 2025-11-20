#include <avr/io.h>
#include <avr/interrupt.h>
volatile char receivedByte;
#define F_CPU 16000000UL // Clock speed of Arduino Uno
#define BAUD 115200
#define MY_UBRR F_CPU/16/BAUD-1
char input;

unsigned char USART_Receive(void) {
  // Wait for data to be received
  while (!(UCSR0A & (1 << RXC0)));
  // Get and return received data from buffer
  return UDR0;
}


void USART_Transmit(unsigned char data) {
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  // Put data into buffer, sends the data
  UDR0 = data;
}

void USART_Init(unsigned int ubrr) {
  // Set baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  // Set frame format: 8 data bits, 1 stop bit, no parity
  UCSR0C = (3 << UCSZ00);
}

ISR(USART_RX_vect) {
  receivedByte = UDR0;  // Read the received byte from the USART data register
  // Handle the byte (e.g., store it, set flags, etc.)
}

void setup() {
 noInterrupt();
  USART_Init(MY_UBRR);
  UCSR0B |= (1 << RXCIE0);  // Enable the USART Receive Complete interrupt
  sei();  // Enable global interrupts
}

void loop() {
  // Main code here. The ISR will run independently when a byte is received.
  input=USART_Receive();
  USART_Transmit(input);  
}
