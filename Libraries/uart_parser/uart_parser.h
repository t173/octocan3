#ifndef UART_PARSER_H_
#define UART_PARSER_H_
#include "main.h"
#include "stdint.h"
// To define ring buffer size, add the following marcro define in main.h file.
// #define RING_BUFF_SIZE (22)

// Increment and decrement ring buffer index.
#define INCREMENT(INDEX)  ( (++INDEX>=RING_BUFF_SIZE)?(INDEX=0):(INDEX) )
#define DECREMENT(INDEX)  ( (--INDEX<=0U)?(INDEX=0):(INDEX) )

typedef struct
{
  UART_HandleTypeDef *huart;      // 
  // Ring buffer related.
  int8_t write; // ring write pointer.
  int8_t read;  // ring read pointer.
  uint8_t length; // length of a continuous write operation.
  uint8_t ready;
  char ring[RING_BUFF_SIZE];
  const char Commands[CMD_SIZE][CMD_LEN_MAX];
} uart_parser_TypeDef;


uint8_t ring_add(uart_parser_TypeDef *hparser);
void ring_clear_index(uart_parser_TypeDef *hparser);
int8_t parser(uart_parser_TypeDef *hparser);

#endif
