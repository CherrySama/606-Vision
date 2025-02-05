#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    /* 五个参量 fd打开文件 speed设置波特率 bit数据位设置   neent奇偶校验位 stop停止位 */
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    switch (nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
int open_port(int fd, int comport)
{
    /* fd 打开串口 comport表示第几个串口 */
    char* dev[] = { "/dev/ttyS0","/dev/ttyS1","/dev/ttyS2" };
    long vdisable;
    if (comport == 1)
    {
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd) {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
            printf("open ttyS0 .....\n");
    }
    if (fcntl(fd, F_SETFL, 0) < 0)
        printf("fcntl failed!\n");
    else
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    if (isatty(STDIN_FILENO) == 0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    printf("fd-open=%d\n", fd);
    return fd;
}
int main(void)
{
    int fd;
    int  i;
    int nread;
    char buff[3];
    buff[0] = 0xaa;
    buff[3] = 0xbb;
     for(int j = 1; j < 3 ; j++)
     {
	 buff[j] = 0x01;
     }
   // char buff[] = "Hello\n";
    fd = open_port(fd, 1);
    if (fd < 0) {
        perror("open_port error");
        return 0;
    }
    i = set_opt(fd, 115200, 8, 'N', 1);
    if (i < 0) {
        perror("set_opt error");
        return 0;
    }
    printf("fd=%d\n", fd);
    //    fd=3;
    nread = read(fd,buff, 8);
    while(1){
    sleep(1);
    	write(fd,buff,8);
	for(int j = 0; j < 3; j++) {
    		printf("nread=%d,%x\n", nread, buff[j]);
	}
    }
    close(fd);
    return 0;

}
