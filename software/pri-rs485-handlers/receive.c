/* receive.c - command-line utility that receives a string of text
 * from RS-485 line
 *
 * Copyright © 2016 Sergey Portnov
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

//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013

// Access from ARM Running Linux
 
#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define GPIO_PIN 18

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <termios.h>		//Used for UART
#include <string.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
 
// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
 
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
 
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

//------------------------- SETUP USART 0 ----- --------------------
//----- At bootup, pins 8 and 10 are already set to UART0_TXD,
//----- UART0_RXD (ie the alt0 function) respectively
int uart0_filestream = -1;

void setup_io();
 
void printButton(int g)
{
  if (GET_GPIO(g)) // !=0 <-> bit is 1 <- port is HIGH=3.3V
    printf("Button pressed!\n");
  else // port is LOW=0V
    printf("Button released!\n");
}
 
int main(int argc, char **argv)
{
  int rx_length;
  unsigned char rx_buffer[256];
   
  // Set up gpi pointer for direct register access
  setup_io();
 
  // Switch GPIO 7..11 to output mode
 
 /************************************************************************\
  * You are about to change the GPIO settings of your computer.          *
  * Mess this up and it will stop working!                               *
  * It might be a good idea to 'sync' before running this program        *
  * so at least you still have your code changes written to the SD-card! *
 \************************************************************************/
 
  // Set GPIO pin 18 to output
  INP_GPIO(GPIO_PIN); // must use INP_GPIO before we can use OUT_GPIO
  OUT_GPIO(GPIO_PIN);

  // set pin connected to RE/DE inputs of MAX485 to 0
  GPIO_CLR = 1<<GPIO_PIN;


  //----- CHECK FOR ANY RX BYTES -----
  if (uart0_filestream != -1) {
    // Read up to 255 characters from the port if they are there
    rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
    //Filestream, buffer to store in, number of bytes to read (max)
    if (rx_length < 0) {
      exit(3);
      //An error occured (will occur if there are no bytes)
    }
    else if (rx_length == 0) {
	//No data waiting
    }
    else {
      //Bytes received
      rx_buffer[rx_length] = '\0';
      printf("%s", rx_buffer);
    }
  }

  return 0;
 
} // main
 
 
//
// Set up a memory regions to access GPIO
//
void setup_io()
{
  int  mem_fd;
  void *gpio_map;
  struct termios options;

  /* open /dev/mem */
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    printf("can't open /dev/mem \n");
    exit(-1);
  }
  
  /* mmap GPIO */
  gpio_map = mmap(
		  NULL,             //Any adddress in our space will do
		  BLOCK_SIZE,       //Map length
		  PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
		  MAP_SHARED,       //Shared with other processes
		  mem_fd,           //File to map
		  GPIO_BASE         //Offset to GPIO peripheral
		  );
  
  close(mem_fd); //No need to keep mem_fd open after mmap
  
  if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
  }
  
  // Always use volatile pointer!
  gpio = (volatile unsigned *)gpio_map;  

  //OPEN THE UART
  //The flags (defined in fcntl.h):
  //	Access modes (use 1 of these):
  //		O_RDONLY - Open for reading only.
  //		O_RDWR - Open for reading and writing.
  //		O_WRONLY - Open for writing only.
  //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking
  //	mode. When set read requests on the file can return
  //	immediately with a failure status if there is no input
  //	immediately available (instead of blocking). Likewise, write
  //	requests can also return immediately with a failure status if
  //	the output can't be written immediately.
  //
  //	O_NOCTTY - When set and path identifies a terminal
  //	device, open() shall not cause the terminal device to
  //	become the controlling terminal for the process.
  uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		
  //Open in non blocking read/write mode
  if (uart0_filestream == -1) {
    //ERROR - CAN'T OPEN SERIAL PORT
    printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    exit(2);
  }

  //CONFIGURE THE UART
  // The flags (defined in /usr/include/termios.h - see
  //http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400,
  //	B57600, B115200, B230400, B460800, B500000, B576000, B921600,
  //	B1000000, B1152000, B1500000, B2000000, B2500000, B3000000,
  //	B3500000, B4000000
  //	CSIZE:- CS5, CS6, CS7, CS8
  //	CLOCAL - Ignore modem status lines
  //	CREAD - Enable receiver
  //	IGNPAR = Ignore characters with parity errors
  //	ICRNL - Map CR to NL on input (Use for ASCII comms where you
  //	want to auto correct end of line characters - don't use for
  //	bianry comms!)
  //	PARENB - Parity enable
  //	PARODD - Odd parity (else even)

  tcgetattr(uart0_filestream, &options);
  options.c_cflag = B9600 | CS8 | CLOCAL | CREAD | PARENB; //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(uart0_filestream, TCIFLUSH);
  tcsetattr(uart0_filestream, TCSANOW, &options);
} // setup_io
