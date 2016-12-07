#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define DISPLAY_STRING

struct opts{

    int baudrate;
    char *port;
    int buff;
    int display;
}opts_1;

int baudrate_select(int x);
int set_interface_attribs(int fd, int speed);
void set_mincount(int fd, int mcount);


int main(int argc, char *argv[])
{
    int opt;
    while((opt=getopt(argc,argv,"p:b:r:d:"))!=-1){
        switch (opt) {
        case 'p':
            opts_1.port=optarg;
            break;
        case 'b':
            if(baudrate_select(atoi(optarg))==-1){
                printf("Wrong baudrate\n");
            exit(EXIT_FAILURE);
            }
            break;
        case 'r':
            opts_1.buff=atoi(optarg);
            break;
        case 'd':
            opts_1.display=atoi(optarg);
            break;
        default:
           //printf("Invalid parameters\n");
           //printf("Usage -p port -b baudrate -r buffer -s 1 display string 0 diplay hex\n");
           // exit(EXIT_FAILURE);
            break;
        }
    }

    printf("port=%s, BR=%d",opts_1.port,opts_1.baudrate);
    char *portname = opts_1.port;
    int fd;
    int wlen;

    fd = open(portname, O_RDWR | O_NOCTTY | O_ASYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, opts_1.baudrate);

    //set_mincount(fd, 0);                /* set to pure timed read */

    /* simple output */
    wlen = write(fd, "Hello!\n", 7);
    if (wlen != 7) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */



    /* simple noncanonical input */
    do {
        unsigned char buf[opts_1.buff];
        int rdlen;

        rdlen = read(fd, buf, sizeof(buf) - 1);

            if (rdlen > 0) {
                if(opts_1.display==1){
                buf[rdlen] = 0;
                printf("%s", buf);
            }
            else{
                unsigned char   *p;
                printf("Read %d:", rdlen);
                for (p = buf; rdlen-- > 0; p++)
                    printf(" 0x%x", *p);
                printf("\n");
            }
        } else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
        /* repeat read to get full message */
    } while (1);
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

int baudrate_select(int x){
    if(x==0) opts_1.baudrate=B0;
    else if(x==50) opts_1.baudrate=B50;
    else if(x==75) opts_1.baudrate=B75;
    else if(x==110) opts_1.baudrate=B110;
    else if(x==134) opts_1.baudrate=B134;
    else if(x==150) opts_1.baudrate=B150;
    else if(x==200) opts_1.baudrate=B200;
    else if(x==300) opts_1.baudrate=B300;
    else if(x==600) opts_1.baudrate=B600;
    else if(x==1200) opts_1.baudrate=B1200;
    else if(x==1800) opts_1.baudrate=B1800;
    else if(x==2400) opts_1.baudrate=B2400;
    else if(x==4800) opts_1.baudrate=B4800;
    else if(x==9600) opts_1.baudrate=B9600;
    else if(x==19200) opts_1.baudrate=B19200;
    else if(x==38400) opts_1.baudrate=B38400;
    else if(x==57600) opts_1.baudrate=B57600;
    else if(x==115200) opts_1.baudrate=B115200;
    else if(x==230400) opts_1.baudrate=B230400;
    else
        return -1;
  return 0;
}
