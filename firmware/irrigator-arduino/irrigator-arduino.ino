/* Irrigator (Arduino version) firmware source code
 * Copyright © 2016-2017 Sergey Portnov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Authors: Sergey Portnov <sergius256@gmail.com>
 */

/* Parser #defines:

   ADDR: Each device gets its own unique address (like MAC addresses
   of network adapters) built of 4 bytes. First reflects device class
   and other are its serial number.

   NUM_PARSER_FUNCTIONS: constant needed to know how many commands the
   device will react on.

   INIT_LEN: Short description - command's preamble length. Long
   description: each command addressed to the specific device consists
   of its address and the command itself. It is needed when we unite
   UARTs of more than one devices in a common network built with
   Master-Slave principle in mind. In this case Master's commands and
   slaves answers are broadcasted causing troubles in making
   difference between slaves. Thus we just simply add preamble to each
   transaction. For example, "DEV01000001AT\n" transaction consists of
   "DEV01000001" - device address and "AT" - the command
   itself. INIT_LEN constant contains the length of device address
   string.

   BUFFER_LEN: UART recieves data byte-by-byte which is not good for
   string- or phrase-based protocol. So we just put all recieved bytes
   in the buffer (char *rxbuffer) and start to analyze it only when a
   special marker - end of line symbol - will be recieved by UART. We
   postulate here that commands are sent in 1-byte ASCII
   encoding. BUFFER_LEN points to the length of command buffer.

   COMMAND_NAME_LEN: maximum command length in previous example. We
   put it here to simplify memory allocation. After all, we have a
   microcontroller chip with fixed amount of memory and malloc()
   possibly costs more bytes than all fixed-legth arrays together.
 */

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
#define NUM_PARSER_FUNCTIONS 3
#endif

#ifndef COMMAND_NAME_LEN
#define COMMAND_NAME_LEN 8
#endif

////////////////////////////////////////////////////////////////////////////
// Program #defines

#define NUM_CHANNELS 8
// Change to 11 after changing the valve
#define HIGHEST_PIN 10
#define MIN_SIGNAL 550
#define MAX_SIGNAL 950
#define MIN_INIT_SIGNAL 0
#define MAX_INIT_SIGNAL 1024

////////////////////////////////////////////////////////////////////////////
// Parser data

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

////////////////////////////////////////////////////////////////////////////
// Program data

signed char DoAttention(void);
signed char DoSetBoudaries(void);
signed char DoGetValues(void);

int MinSignal[NUM_CHANNELS],CurSignal[NUM_CHANNELS],MaxSignal[NUM_CHANNELS];
short int ChannelState[NUM_CHANNELS];

////////////////////////////////////////////////////////////////////////////
// Parser functions
void SetDeviceID(void)
{
  /*
    This function should be called before all others to initialize
    device_id pointer.
  */
  device_id[0] ='D';
  device_id[1] ='E';
  device_id[2] ='V';
  device_id[3] ='0'+(ADDR>>28)%0x10;
  device_id[4] ='0'+(ADDR>>24)%0x10;
  device_id[5] ='0'+(ADDR>>20)%0x10;
  device_id[6] ='0'+(ADDR>>16)%0x10;
  device_id[7] ='0'+(ADDR>>12)%0x10;
  device_id[8] ='0'+(ADDR>>8)%0x10;
  device_id[9] ='0'+(ADDR>>4)%0x10;
  device_id[10]='0'+ADDR%0x10;
  device_id[11]=0;
}

signed char IsTransmissionToOurs(void)
{
  /*
    This function is needed to check if the last command addressing
    our device or not. If yes, device identifier is cleared out from
    buffer to simplify further parsing. If no, buffer gets cleared
    out.
  */
  unsigned char i;
  
  if(strncasecmp(buffer,device_id,INIT_LEN))
    {
      buffer[0]=0;
      buffer_cur_len=0;
      return 0;
    }
  
  buffer_cur_len=strnlen(buffer,BUFFER_LEN); // can be commented out due to program logic
  for(i=0;i<buffer_cur_len-INIT_LEN;i++)
    buffer[i]=buffer[i+INIT_LEN];
  buffer[i]=0;
  buffer_cur_len=i;
  return -1;
}

THandler Parser(void)
{
  /*
    This function should be called if transmitted text is addressed to
    our device.
  */
  unsigned char i,j;
  
  for(j=0;j<NUM_PARSER_FUNCTIONS;j++)
    {
      if(strncasecmp(buffer,ParserFunctions[j].name,ParserFunctions[j].name_len)==0)
	{
	  for(i=0;i<buffer_cur_len-ParserFunctions[j].name_len;i++)
	    buffer[i]=buffer[i+ParserFunctions[j].name_len];
	  buffer[i]=0;
	  buffer_cur_len=i;
	  return ParserFunctions[j].handler;
	}
    }
  return NULL;
}

/*
  This is needed to maintain two buffers for received data.
*/

void CopyFromRXtoParser(void)
{
  unsigned char i;
  
  //Serial.println(rxbuffer_cur_len);
  //Serial.println(rxbuffer);
  for(i=0;i<rxbuffer_cur_len;i++)
    {
      buffer[i]=rxbuffer[i];
    }
  if(i<BUFFER_LEN)
    buffer[i]=0;
  buffer_cur_len=rxbuffer_cur_len;
  rxbuffer_cur_len=0;
  //Serial.println(buffer);
}

/////////////////////////////////////////////////////////////////////
// Program functions - command handlers

signed char DoAttention(void)
{
  Serial.print(device_id);
  Serial.println("OK");
  return 0;
}

signed char DoSetBoudaries(void)
{
  // Command sould be DEVxxxxxxxxSETcMINaMAXb\n
  // When calling this function, buffer will contain "cMINaMAXb\n"
  int i, next_token_pos, multiplier=1, c=0, a=0, b=0;
  
  // Go to the last digit of channel number:
  for(i=0;i<buffer_cur_len && buffer[i] >= '0' && buffer[i] <= '9';i++);
  next_token_pos = i;
  i--;
  for(;i>=0;i--)
    {
      c+=(buffer[i]-'0')*multiplier;
      multiplier*=10;
    }
  
  // Shift buffer left
  for(i=0;i<buffer_cur_len-next_token_pos;i++)
    buffer[i]=buffer[i+next_token_pos];
  buffer[i]=0;
  buffer_cur_len=i;
  
  if(strncasecmp(buffer,"MIN",3)==0)
    {
      for(i=0;i<buffer_cur_len-3;i++)
	buffer[i]=buffer[i+3];
      buffer[i]=0;
      buffer_cur_len=i;
      
      // Go to the last digit of minimal value:
      for(i=0;i<buffer_cur_len && buffer[i] >= '0' && buffer[i] <= '9';i++);
      next_token_pos = i;
      i--;
      multiplier=1;
      for(;i>=0;i--)
	{
	  a+=(buffer[i]-'0')*multiplier;
	  multiplier*=10;
	}
      
      // Shift buffer left
      for(i=0;i<buffer_cur_len-next_token_pos;i++)
	buffer[i]=buffer[i+next_token_pos];
      buffer[i]=0;
      buffer_cur_len=i;
    }
  else
    return -1;
  
  if(strncasecmp(buffer,"MAX",3)==0)
    {
      for(i=0;i<buffer_cur_len-3;i++)
	buffer[i]=buffer[i+3];
      buffer[i]=0;
      buffer_cur_len=i;
      
      // Go to the last digit of minimal value:
      for(i=0;i<buffer_cur_len && buffer[i] >= '0' && buffer[i] <= '9';i++);
      next_token_pos = i;
      i--;
      multiplier=1;
      for(;i>=0;i--)
	{
	  b+=(buffer[i]-'0')*multiplier;
	  multiplier*=10;
	}
      
      // Shift buffer left
      for(i=0;i<buffer_cur_len-next_token_pos;i++)
	buffer[i]=buffer[i+next_token_pos];
      buffer[i]=0;
      buffer_cur_len=i;
    }
  else
    return -1;
  
  // Ok, now we have channel number, min and max signal values
  MinSignal[c]=a;
  MaxSignal[c]=b;
  Serial.print(device_id);
  Serial.println("OK");
  return 0;
}

signed char DoGetValues(void)
{
  int i;
  
  Serial.print(device_id);
  
  for(i=0;i<NUM_CHANNELS;i++)
    {
      // Possibly not the best solution because it can add some
      // undefined time to any main operation, but we don't use
      // Arduino to control nuclear power plant.
      Serial.print("C");
      Serial.print(i);
      Serial.print("MIN");
      Serial.print(MinSignal[i]);
      Serial.print("MAX");
      Serial.print(MaxSignal[i]);
      Serial.print("CUR");
      Serial.print(CurSignal[i]);
    }
  Serial.println("OK");
  return 0;
}

/////////////////////////////////////////////////////////////////////
// Program functions - main

void setup(void)
{
  int i;
  
  Serial.begin(9600);
  for(i=HIGHEST_PIN;i>HIGHEST_PIN - NUM_CHANNELS;i--)
    pinMode(i,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  
  ParserFunctions[0].name_len = 2;
  sprintf(ParserFunctions[0].name,"AT");
  ParserFunctions[0].handler = DoAttention;
  
  ParserFunctions[1].name_len = 3;
  sprintf(ParserFunctions[1].name,"SET");
  ParserFunctions[1].handler = DoSetBoudaries;
  
  ParserFunctions[2].name_len = 3;
  sprintf(ParserFunctions[2].name,"GET");
  ParserFunctions[2].handler = DoGetValues;
  
  // Initialize MinSignal, CurSignal, MaxSignal and ChannelState
  // MinSignal and MaxSignal should be lower and higher boundaries
  // given by ADC, ChannelState should say the device to do nothing
  for(i=0;i<NUM_CHANNELS;i++)
    {
      MinSignal[i]=MIN_INIT_SIGNAL;
      CurSignal[i]=analogRead(i);
      MaxSignal[i]=MAX_INIT_SIGNAL;
      ChannelState[i]=LOW;
      digitalWrite(HIGHEST_PIN-i,LOW);
    }
  SetDeviceID();
}

void loop()
{
  // put your main code here, to run repeatedly:
  THandler hndl;
  int i;
  
  if(transmission_ready_flag)
    {
      if(IsTransmissionToOurs())
	{
          if((hndl=Parser())==NULL)
	    {
	      // Parser didn't find proper function
	      Serial.print(device_id);
	      Serial.println("ERROR_NO_FUNCTION");
	    }
	  else
	    if(hndl())
	      {
		// Called dunction handler dropped with error
		Serial.print(device_id);
		Serial.println("ERROR");
	      }
	}
      
      // Final cleanup
      buffer[0]=0;
      buffer_cur_len=0;
      transmission_ready_flag = 0;
    }
  
  // After all the etiquette is done, we can do our main job
  for(i=0;i<NUM_CHANNELS;i++)
    {
      CurSignal[i]=analogRead(i);
    }
  for(i=0;i<NUM_CHANNELS;i++)
    {
      if(CurSignal[i]>MaxSignal[i]) // Sensor is dry
	ChannelState[i]=HIGH; //
      if(CurSignal[i]<MinSignal[i]) // Sensor is wet
	ChannelState[i]=LOW;
    }
  for(i=0;i<NUM_CHANNELS;i++)
    digitalWrite(HIGHEST_PIN-i,ChannelState[i]);
}

// Serial interrupt handler needed to put recieved data into rxbuffer[],
// watch for '\n' in command and expose command to the rest of program when
// needed.
void serialEvent(void)
{
  while(Serial.available() && (transmission_ready_flag == 0))
    {
      rxbuffer[rxbuffer_cur_len] = (char)Serial.read();
      if((rxbuffer[rxbuffer_cur_len] == '\n') || (rxbuffer_cur_len >= BUFFER_LEN))
	{
	  CopyFromRXtoParser();
	  transmission_ready_flag = 255;
	}
      else
	rxbuffer_cur_len++;
    }
}
