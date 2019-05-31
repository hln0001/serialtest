#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <termios.h>

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
  }
  return(fd);
}

void sendPacket(int fd)
{
  unsigned char shtpData[] = {0x15,0x00,0x02,0x00,0xFD,0x01,0,0,0,0x60,0xEA,0,0,0,0,0,0,0,0,0,0};
  //unsigned char shtpData[] = {0x06,0x00,0x02,0x01,0xFE,0x14};

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

  cfsetispeed(&PortSettings, 3000000);
  cfsetospeed(&PortSettings, 3000000);

  PortSettings.c_cflag &= ~PARENB;

  PortSettings.c_cflag &= ~CSTOPB;

  PortSettings.c_cflag &= ~CSIZE;
  PortSettings.c_cflag |= CS8;

  PortSettings.c_cflag &= ~CRTSCTS;
  PortSettings.c_cflag |= CREAD | CLOCAL;
  PortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
  PortSettings.c_lflag |= ICANON;

  PortSettings.c_oflag &= ~OPOST;

  tcsetattr(fd, TCSANOW, &PortSettings);

  sendPacket(fd);

  char read_buffer[256];
  int bytes_read = 0;

  bytes_read = read(fd, read_buffer, 256);

  std::cout << "# of Bytes Read: " << bytes_read << '\n';

  for(int i = 0; i < bytes_read; i++)
  {
    printf("%02X\n", read_buffer[i]);
  }

  close(fd);
  return 0;
}
