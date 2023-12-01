#include "main.h"
#include "uart_parser.h"
#include <string.h>
// Define CMD_SIZE AND CMD_LEN_MAX in main.h file.
// CMD_SIZE: Number of commands
// CMD_LEN_MAX: Maximum length of a single command.
// Declare Commands array and States enum in main.c.

//extern enum States;


// Fetch characters from UART. Not more than 9 (8+1) from UART hardware can be handled.
uint8_t ring_add(uart_parser_TypeDef *hparser)
{
	char temp;
	if(hparser->length >= RING_BUFF_SIZE)
		return 1;		// Ring buffer full before write.
	else
	{
		while(READ_BIT(hparser->huart->Instance->ISR, USART_ISR_RXNE_RXFNE) != 0 )
//		while(READ_BIT(USART2->ISR, USART_ISR_RXNE_RXFNE) != 0 )
		{
			temp = (char)hparser->huart->Instance->RDR;
			if( temp == '\b')		// if backspace
			{
				if(hparser->length != 0)
				{
					hparser->length--;
					if(hparser->write == 0)
						hparser->write = RING_BUFF_SIZE - 1;
					else
						hparser->write--;
				}
				hparser->huart->Instance->TDR = '\b';
				hparser->huart->Instance->TDR = ' ';
				hparser->huart->Instance->TDR = '\b';
			}
			else
			{
				if(temp == '\r')	// if enter
				{
					hparser->ring[hparser->write] = '\r';
					hparser->huart->Instance->TDR = '\r';
					hparser->huart->Instance->TDR = '\n';
					hparser->ready = 1;
				}
				else		// all other chars
				{
					hparser->ring[hparser->write] = temp;
					hparser->huart->Instance->TDR = (uint32_t)hparser->ring[hparser->write];
				}
				INCREMENT(hparser->write);
				hparser->length++;
				if(hparser->length > RING_BUFF_SIZE)
					return 2;		// Ring buffer over flown during writing operation.
			}
		}
		return 0;		// Ring buffer write operation successful.
	}
}

void ring_clear_index(uart_parser_TypeDef *hparser)
{
	hparser->length = 0;
	hparser->read = hparser->write;
}
// Simple command parser. argument is the current read position of the ring buffer.
// This way, it handles the situation that when a command wraps back to beginning
// position of the ring buffer, the parser still compares the received string with
// the predefined commands.
// the position variable needs to be stored at the beginning of the function, in
// case it has been incremented during one failed command parsing iteration, and
// been used again in another iteration. which causes the parser cannot differentiate
// commands such as "TMP102" and "TEMP".
int8_t parser(uart_parser_TypeDef *hparser)
{
	uint8_t len,ret,i;
	uint8_t position_init = hparser->read;
	uint8_t position = 0;
//	extern const char Commands[CMD_SIZE][CMD_LEN_MAX];
	// This for loop compares received buffer against different commands.

	for(i=0;i<CMD_SIZE;i++)
	{
		ret = 1;
		position = position_init;
		// This loop compares the content of a command.
		for(len=0;len<strlen(hparser->Commands[i]);len++)
		{
			if(hparser->Commands[i][len] == hparser->ring[position])
			{
				INCREMENT(position);
			}
			else
			{
				ret = 0;
				break;
			}
		}
		// At the end of each content comparison, if a ring buffer
		// matches a command defined in Commands[], then ret=1
		// and return the number of that command.
		// here only put the read point at the character right after a command text.
		// such as CLEAR, the read is pointed at /r, instead of the new command.
		if(ret == 1)
		{
			for(len=0;len<strlen(hparser->Commands[i]);len++)
			{
				INCREMENT(hparser->read);
				DECREMENT(hparser->length);
			}
			return i;
		}
	}
	return -1;
}
