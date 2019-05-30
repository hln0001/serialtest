#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <termios.h>

char shtpData[21];

int open_port()
{
  int fd;

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY );

  if(fd == -1)
  {
    perror("open_port: Unable to open /dev/ttyUSB0");
  }
  else
  {
    std::cout << "Port opened " << fd << '\n';
    fcntl(fd, F_SETFL, 0);
    //std::cout << "fcntl\n";
  }
  return(fd);
}

void sendPacket(int fd)
{
  shtpData[0] = 0x15;
  shtpData[1] = 0x00;
  shtpData[2] = 0x02;
  shtpData[3] = 0x01;
  shtpData[4] = 0xFD;	               //Set feature command. Reference page 55
  shtpData[5] = 0x01;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[6] = 0;								   //Feature flags
  shtpData[7] = 0;								   //Change sensitivity (LSB)
  shtpData[8] = 0;								   //Change sensitivity (MSB)
  shtpData[9] = 0x60;                //Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtpData[10] = 0xEA;               //Report interval
  shtpData[11] = 0;                  //Report interval
  shtpData[12] = 0;                  //Report interval (MSB)
  shtpData[13] = 0;								   //Batch Interval (LSB)
  shtpData[14] = 0;								   //Batch Interval
  shtpData[15] = 0;								   //Batch Interval
  shtpData[16] = 0;								   //Batch Interval (MSB)
  shtpData[17] = 0;	                 //Sensor-specific config (LSB)
  shtpData[18] = 0;	                 //Sensor-specific config
  shtpData[19] = 0;	                 //Sensor-specific config
  shtpData[20] = 0;	                 //Sensor-specific config (MSB)

  int n = write(fd,shtpData,21);
  if(n < 0)
  {
    fputs("write failed\n", stderr);
    std::cout << n;
  }
  else std::cout << "something got written: " << n << " bytes\n";
  return;
}

int main()
{
  int fd = open_port();

  struct termios PortSettings;

  tcgetattr(fd, &PortSettings);
  //std::cout << "get attr\n";

  cfsetispeed(&PortSettings, 3000000);
  cfsetospeed(&PortSettings, 3000000);
  //std::cout << "set io speed\n";

  PortSettings.c_cflag &= ~PARENB;

  PortSettings.c_cflag &= ~CSTOPB;

  PortSettings.c_cflag &= ~CSIZE;
  PortSettings.c_cflag |= CS8;

  PortSettings.c_cflag &= ~CRTSCTS;
  PortSettings.c_cflag |= CREAD | CLOCAL;
  PortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
  PortSettings.c_iflag |= ICANON;

  PortSettings.c_oflag &= ~OPOST;

  tcsetattr(fd, TCSANOW, &PortSettings);
  //std::cout << "set attr\n";

  sendPacket(fd);

  char read_buffer[256];
  int bytes_read = 0;

  bytes_read = read(fd, &read_buffer, 256);

  std::cout << "# of Bytes Read: " << bytes_read << '\n';

  for(int i = 0; i < bytes_read; i++)
  {
    printf("%02X\n", read_buffer[i]);
  }

  close(fd);
  return 0;
}
