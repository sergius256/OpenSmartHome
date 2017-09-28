/* Window blind lift (Arduino version) firmware source code
   Copyright © 2017 Sergey Portnov

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see http://www.gnu.org/licenses/ .

   Authors: Sergey Portnov <sergius256@gmail.com>
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
#define ADDR 0x02000001
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
// Parser data

typedef signed char (*THandler)(void);

typedef struct {
  unsigned char name_len;
  char name[COMMAND_NAME_LEN];
  THandler handler;
} TParserConfig;

char buffer[BUFFER_LEN], rxbuffer[BUFFER_LEN];
char device_id[INIT_LEN + 1];

TParserConfig ParserFunctions[NUM_PARSER_FUNCTIONS];
unsigned char buffer_cur_len = 0, rxbuffer_cur_len = 0, transmission_ready_flag = 0;

void SetDeviceID(void);
signed char IsTransmissionToOurs(void);
THandler Parser(void);
void CopyFromRXtoParser(void);

////////////////////////////////////////////////////////////////////////////
// Program data

signed char DoAttention(void);
signed char DoSetPercentOpen(void);
signed char DoGetPercentOpen(void);

/*
  Global variables:

  MayStopFlag - when set to nonzero, allows to stop movement by
               pressing any button
  timeFull    - milliseconds to go from one position to another
  timeCurrent - milliseconds from upper to current position

  DEBOUNCE_MAGIC - debounce inactivity period, milliseconds
*/

int MayStopFlag = 0;
unsigned long int timeFull = 0, timeCurrent = 0;
void GoDown(unsigned long int t);
void GoUp(unsigned long int t);

#define DEBOUNCE_MAGIC 200

////////////////////////////////////////////////////////////////////////////
// Parser functions
void SetDeviceID(void)
{
  /*
    This function should be called before all others to initialize
    device_id pointer.
  */
  device_id[0] = 'D';
  device_id[1] = 'E';
  device_id[2] = 'V';
  device_id[3] = '0' + (ADDR >> 28) % 0x10;
  device_id[4] = '0' + (ADDR >> 24) % 0x10;
  device_id[5] = '0' + (ADDR >> 20) % 0x10;
  device_id[6] = '0' + (ADDR >> 16) % 0x10;
  device_id[7] = '0' + (ADDR >> 12) % 0x10;
  device_id[8] = '0' + (ADDR >> 8) % 0x10;
  device_id[9] = '0' + (ADDR >> 4) % 0x10;
  device_id[10] = '0' + ADDR % 0x10;
  device_id[11] = 0;
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

  if (strncasecmp(buffer, device_id, INIT_LEN))
  {
    buffer[0] = 0;
    buffer_cur_len = 0;
    return 0;
  }

  buffer_cur_len = strnlen(buffer, BUFFER_LEN); // can be commented out due to program logic
  for (i = 0; i < buffer_cur_len - INIT_LEN; i++)
    buffer[i] = buffer[i + INIT_LEN];
  buffer[i] = 0;
  buffer_cur_len = i;
  return -1;
}

THandler Parser(void)
{
  /*
    This function should be called if transmitted text is addressed to
    our device.
  */
  unsigned char i, j;

  for (j = 0; j < NUM_PARSER_FUNCTIONS; j++)
  {
    if (strncasecmp(buffer, ParserFunctions[j].name, ParserFunctions[j].name_len) == 0)
    {
      for (i = 0; i < buffer_cur_len - ParserFunctions[j].name_len; i++)
        buffer[i] = buffer[i + ParserFunctions[j].name_len];
      buffer[i] = 0;
      buffer_cur_len = i;
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
  for (i = 0; i < rxbuffer_cur_len; i++)
  {
    buffer[i] = rxbuffer[i];
  }
  if (i < BUFFER_LEN)
    buffer[i] = 0;
  buffer_cur_len = rxbuffer_cur_len;
  rxbuffer_cur_len = 0;
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

signed char DoSetPercentOpen(void)
{
  // Command sould be DEVxxxxxxxxSETppp\n
  // When calling this function, buffer will contain "ppp\n"
  int i, next_token_pos, multiplier = 1, p = 0;
  unsigned long int newTimeCurrent;

  // Go to the last digit of channel number:
  for (i = 0; i < buffer_cur_len && buffer[i] >= '0' && buffer[i] <= '9'; i++);
  next_token_pos = i;
  i--;
  for (; i >= 0; i--)
  {
    p += (buffer[i] - '0') * multiplier;
    multiplier *= 10;
  }

  // Ok, now we have percent value

  newTimeCurrent = timeFull * p / 100;
  if (newTimeCurrent < timeCurrent)
    GoUp(timeCurrent - newTimeCurrent);
  if (newTimeCurrent > timeCurrent)
    GoDown(newTimeCurrent - timeCurrent);

  Serial.print(device_id);
  Serial.println("OK");
  return 0;
}

signed char DoGetPercentOpen(void)
{
  int pos;

  pos = timeCurrent / timeFull * 100;
  Serial.print(device_id);
  Serial.print("POS");
  Serial.println(pos);
  return 0;
}



void setup() {
  unsigned long int t1, t2, t3;

  Serial.begin(9600);
// Pins 11 and 12 are connected to motorshield
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
// Pins 7 to 10 are connected to position sensors and pushbuttons.
// Active state is LOW.
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
// LED pin
  pinMode(13, OUTPUT);
// Turning off motor
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);

// Parser setup
  ParserFunctions[0].name_len = 2;
  sprintf(ParserFunctions[0].name, "AT");
  ParserFunctions[0].handler = DoAttention;

  ParserFunctions[1].name_len = 3;
  sprintf(ParserFunctions[1].name, "SET");
  ParserFunctions[1].handler = DoSetPercentOpen;

  ParserFunctions[2].name_len = 3;
  sprintf(ParserFunctions[2].name, "GET");
  ParserFunctions[2].handler = DoGetPercentOpen;

  SetDeviceID();

// Measuring blind full movement time
  t1 = millis();
  GoDown(0xFFFFFF);
  t2 = millis();
  GoUp(0xFFFFFF);
  t3 = millis();
  timeFull = t3 - t2;
  timeCurrent = t3 - 2 * t2 - t1; // timeFull - (t2-t1)
// Returning blind in initial position
  GoDown(timeCurrent);
  MayStopFlag = -1;
}

void loop() {
  /* Testing routines:

    Serial.print(digitalRead(7)); // Right contact
    Serial.print(" ");
    Serial.print(digitalRead(8)); // UP key
    Serial.print(" ");
    Serial.print(digitalRead(9)); // Left contact
    Serial.print(" ");
    Serial.println(digitalRead(10)); // DOWN key

    digitalWrite(11,HIGH); // CW
    delay(1000);
    digitalWrite(11,LOW);
    delay(1000);
    digitalWrite(12,HIGH); // CCW
    delay(1000);
    digitalWrite(12,LOW);
    delay(1000);
  */

  THandler hndl;

  if (transmission_ready_flag)
  {
    if (IsTransmissionToOurs())
    {
      if ((hndl = Parser()) == NULL)
      {
        // Parser didn't find proper function
        Serial.print(device_id);
        Serial.println("ERROR_NO_FUNCTION");
      }
      else if (hndl())
      {
        // Called function handler dropped with error
        Serial.print(device_id);
        Serial.println("ERROR");
      }
    }

    // Final cleanup
    buffer[0] = 0;
    buffer_cur_len = 0;
    transmission_ready_flag = 0;
  }

  // After all the etiquette is done, we can do our main job

  if (digitalRead(10) == 0)
  {
    delay(DEBOUNCE_MAGIC);
    GoDown(timeFull);
  }
  if (digitalRead(8) == 0)
  {
    delay(DEBOUNCE_MAGIC);
    GoUp(timeFull);
  }
}

// Serial interrupt handler needed to put recieved data into rxbuffer[],
// watch for '\n' in command and expose command to the rest of program when
// needed.
void serialEvent(void)
{
  while (Serial.available() && (transmission_ready_flag == 0))
  {
    rxbuffer[rxbuffer_cur_len] = (char)Serial.read();
    if ((rxbuffer[rxbuffer_cur_len] == '\n') || (rxbuffer_cur_len >= BUFFER_LEN))
    {
      CopyFromRXtoParser();
      transmission_ready_flag = 255;
    }
    else
      rxbuffer_cur_len++;
  }
}

void GoDown(unsigned long int t)
{
  unsigned long int tStart;

  tStart = millis();

  if(tStart > tStart + t) // Every 50 days millis() counter overflows
    tStart -= 0xFFFFFFFF;

  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  
  while ((tStart + t) > millis())
  {
    if (digitalRead(7) == 0)
      break;
    if ((MayStopFlag != 0) && (digitalRead(8) == 0))
      break;
    if ((MayStopFlag != 0) && (digitalRead(10) == 0))
      break;
  }
  timeCurrent += millis() - tStart;
  digitalWrite(11, LOW);
  delay(DEBOUNCE_MAGIC);
  if (timeCurrent > timeFull)
    timeCurrent = timeFull;
}

void GoUp(unsigned long int t)
{
  unsigned long int tStart;

  tStart = millis();

  if(tStart > tStart + t) // Every 50 days millis() counter overflows
    tStart -= 0xFFFFFFFF;

  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  while ((tStart + t) > millis())
  {
    if (digitalRead(9) == 0)
      break;
    if ((MayStopFlag != 0) && (digitalRead(8) == 0))
      break;
    if ((MayStopFlag != 0) && (digitalRead(10) == 0))
      break;
  }
  timeCurrent -= millis() - tStart;
  digitalWrite(12, LOW);
  delay(DEBOUNCE_MAGIC);
  if (timeCurrent < 0)
    timeCurrent = 0;
}

