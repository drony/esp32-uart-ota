/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * Benjamin Aigner, 2019 <beni@asterics-foundation.org>
 * 
 * This tool is heavily based on the serial example on stackoverflow:
 * https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 * 
 * Basically, this tool opens a tty port and a binary file.
 * This binary file is transmitted via the tty port to the ESP32,
 * which loads the binary into the flash.
 */


#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

//bytes of one packet transmitted in one piece.
//all corresponding buffers should be at least this size.
#define CHUNKSIZE 32

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}


int main(int argc, char* argv[])
{
  //if set to 1, file is finished & we wait for Serial to finish
  int finished = 0;
  //input buffer
  char buf[CHUNKSIZE];
  //sync bytes, to trigger update
  char sync[6] = {0xC0,0xFF,0xFE,0xAA,0x55,0x90};
  //total size of sent bytes
  int totalsize = 0;
  //wait after first few bytes for ESP32 to open the partition
  int wait = 0;
  
  // check parameter number
  if (argc < 3) {
    printf("Usage: ./update <ttyPort> <filename>\r\n");
    printf("Example: ./update /dev/ttyUSB0 /home/FLipMouse/data.bin\r\n");
    return -1;
  }
  
  //map arguments to file names
  char *portname = argv[1];
  char *filename = argv[2];
  
  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  int fd_bin = open (filename,0);
  
  if (fd < 0)
  {
          printf("Serial: error %d opening %s: %s\r\n", errno, portname, strerror (errno));
          return errno;
  }
  
  if(fd_bin < 0)
  {
          printf("File: error %d opening %s: %s\r\n", errno, filename, strerror (errno));
          return errno;
  }
  
  set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 1);                // set blocking
  
  //write to serial terminal - sync bytes
  write (fd, sync, 6);

  //transmit until EOF
  while(!finished)
  {
    //read one chunk from file
    int ret = read(fd_bin,buf,CHUNKSIZE);
    //is there an error?
    if(ret == -1)
    {
      printf("error %d reading %s: %s\r\n", errno, filename, strerror (errno));
      return errno;
    } else totalsize += ret;
    //check if we reached EOF
    if(ret != CHUNKSIZE)
    {
      printf("Finished reading, total size: %d\r\n",totalsize);
      finished = 1;
    }
    
    //write to serial terminal
    write (fd, buf, ret);
    
    //output written data
    if((totalsize % 2048) == 0) printf("Written: %d\r\n",totalsize);
    
    //wait after the partition header
    //the ESP32 needs a few seconds after opening the partition.
    if((totalsize >= 512) && (wait == 0)) 
    {
      printf("Wrote first chunk, waiting 10s for ESP32 to opening new partition\r\n");
      usleep(10000000);
      wait = 1;
    }
  
    //sleep...
    #if CHUNKSIZE <= 512
    //usleep(CHUNKSIZE*80);
    #endif
    #if CHUNKSIZE >= 512
    usleep(CHUNKSIZE*700);
    #endif
  }
  
  return 0;
}
