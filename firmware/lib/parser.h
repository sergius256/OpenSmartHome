#ifndef __PARSER_H
#define __PARSER_H

#ifndef ADDR
#define ADDR 0x01000001
#endif

#ifndef INIT_LEN
#define INIT_LEN 11
#endif

#ifndef BUFFER_LEN
#define BUFFER_LEN 32 // One command per line
#endif

#ifndef NUM_PARSER_FUNCTIONS
#define NUM_PARSER_FUNCTIONS 1
#endif

#ifndef COMMAND_NAME_LEN
#define COMMAND_NAME_LEN 8
#endif

typedef signed char (*THandler)(void);

typedef struct {
  unsigned char name_len;
  char name[COMMAND_NAME_LEN];
  THandler handler;
} TParserConfig;

char buffer[BUFFER_LEN], rxbuffer[BUFFER_LEN];
char device_id[INIT_LEN+1];

TParserConfig ParserFunctions[NUM_PARSER_FUNCTIONS];
unsigned char buffer_cur_len=0,rxbuffer_cur_len=0,transmission_ready_flag=0;

void SetDeviceID(void);
signed char IsTransmissionToOurs(void);
THandler Parser(void);
void CopyFromRXtoParser(void);

#endif
