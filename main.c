#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>


#define FLASH_START_ADDR    0x08000000
#define FIRMWARE_TAG_ADDR   0x080000ec

#define BURN_DATA_LEN       16


int init_io(void);
int cleanup(void);
void enter_boot_mode(void);
void reset_mcu(void);


static int tty;


int open_serial_port(char *device, speed_t baudrate)
{
    int fd;
    struct termios options;

    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
        return -1;
    else
        fcntl(fd, F_SETFL, 0);

    tcgetattr(fd, &options);

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;        /* one stop bit */
    options.c_cflag &= ~CSIZE;         /* mask the character size bits */
    options.c_cflag |= CS8;            /* select 8 data bits */
    options.c_cflag &= ~CRTSCTS;   /* disable hardware flow control */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* choosing raw input */
    options.c_iflag &= ~(IXON | IXOFF | IXANY);          /* disable software flow control */
    options.c_oflag &= ~OPOST;         /* choosing raw output */
    options.c_cc[VTIME] = 10;
    options.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

int send_cmd(char *cmd, int n)
{
    return write(tty, cmd, n);
}

int get_response(char *resp, int len)
{
    int n, bytes_read = 0;

    while (len) {
        n = read(tty, resp+bytes_read, len);
        if (n <= 0)
            return n;

        len -= n;
        bytes_read += n;
    }

    return bytes_read;
}

int check_mcu_status()
{
    u_char cmd, resp;
    int retry = 5;

    cmd = 0x7f;

    while (retry--) {
        send_cmd(&cmd, 1);
        if (get_response(&resp, 1) == 1 || resp == 0x79)
          return 0;
    }

    return -1;
}

int get_bootloader_version(int *ver)
{
    u_char cmd[2], resp[5];

    cmd[0] = 0x01;
    cmd[1] = 0xfe;
    send_cmd(cmd, 2);

    if (get_response(resp, 5) < 0)
        return -1;

    *ver = resp[1];
    return 0;
}

int get_chip_id(int *id)
{
    u_char cmd[2], resp[5];

    cmd[0] = 0x02;
    cmd[1] = 0xfd;
    send_cmd(cmd, 2);

    if (get_response(resp, 5) < 0)
        return -1;

    *id  = resp[2]*256 + resp[3];
    return 0;
}

int get_firmware_version(int *fv)
{
    u_char cmd[2], resp, data[8];

    cmd[0] = 0x11;
    cmd[1] = 0xee;
    send_cmd(cmd, 2);

    if (get_response(&resp, 1) < 0 || resp != 0x79)
        return -1;

    data[0] = FIRMWARE_TAG_ADDR >> 24;
    data[1] = (FIRMWARE_TAG_ADDR >> 16) & 0xff;
    data[2] = (FIRMWARE_TAG_ADDR >> 8) & 0xff;
    data[3] = FIRMWARE_TAG_ADDR & 0xff;
    data[4] = data[0] ^ data[1] ^ data[2] ^ data[3];

    send_cmd(data, 5);

    if (get_response(&resp, 1) < 0 || resp != 0x79)
        return -1;

    data[0] = 7;
    data[1] = ~7;
    send_cmd(data, 2);

    if (get_response(&resp, 1) < 0 || resp != 0x79)
        return -1;

    if (get_response(data, 8) < 0)
        return -1;

    if (data[0] != 0xa5 || data[1] != 0xa5
        || data[2] != 0xa5 || data[3] != 0xa5)
        memset(fv, 0, 4);
    else
        memcpy(fv, &data[4], 4);

    return 0;
}

int flash_erase()
{
    u_char cmd[2], resp;
    int err = 0;

    cmd[0] = 0x43;
    cmd[1] = 0xbc;
    send_cmd(cmd, 2);

    if (get_response(&resp, 1) < 0 || resp != 0x79)
        return -1;

    cmd[0] = 0xff;
    cmd[1] = 0x00;
    send_cmd(cmd, 2);

_retry:
    if (get_response(&resp, 1) < 0 || resp != 0x79) {
        err++;
        if (err == 20)
            return -1;

        goto _retry;
    }

    return 0;
}

int read_binary_file(FILE *fp, u_char **buf)
{
    struct stat file_stat;
    off_t size;

    fstat(fileno(fp), &file_stat);

    size = file_stat.st_size;
    size += BURN_DATA_LEN;

    *buf = malloc(size);
    if (*buf == NULL)
        return -1;

    if (fread(*buf, file_stat.st_size, 1, fp) == 1)
        return size;
    else {
        free(*buf);
        return -1;
    }
}

int get_update_version(u_char *buf)
{
    if (buf[0x00ec] == 0xa5 && buf[0x00ed] == 0xa5 &&
        buf[0x00ee] == 0xa5 && buf[0x00ef] == 0xa5) {
        return (buf[0x00f3]<<24) + (buf[0x00f2]<<16) +
                (buf[0x00f1]<<8) + buf[0x00f0];
    }

    return -1;
}

int burn_flash(u_char *buf, int size)
{
    int idx = 0, i, cnt;
    u_char cmd[BURN_DATA_LEN+2], resp;

    for (cnt = 0; cnt < size/BURN_DATA_LEN; cnt++) {
        cmd[0] = 0x31;
        cmd[1] = 0xce;
        send_cmd(cmd, 2);

        if (get_response(&resp, 1) < 0 || resp != 0x79)
            return -1;

        cmd[0] = ((FLASH_START_ADDR+idx) >> 24) & 0xff;
        cmd[1] = ((FLASH_START_ADDR+idx) >> 16) & 0xff;
        cmd[2] = ((FLASH_START_ADDR+idx) >> 8) & 0xff;
        cmd[3] = (FLASH_START_ADDR+idx) & 0xff;
        cmd[4] = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3];

        send_cmd(cmd, 5);

        if (get_response(&resp, 1) < 0 || resp != 0x79)
            return -1;

        cmd[0] = BURN_DATA_LEN-1;
        memcpy(cmd+1, buf+idx, BURN_DATA_LEN);
        cmd[BURN_DATA_LEN+1] = 0;
        for (i = 0; i < BURN_DATA_LEN+1; i++)
            cmd[BURN_DATA_LEN+1] ^= cmd[i];

        send_cmd(cmd, BURN_DATA_LEN+2);

        if (get_response(&resp, 1) < 0 || resp != 0x79)
            return -1;

        idx += BURN_DATA_LEN;
    }

    return 0;
}

int main(int argc, char **argv)
{
    int ver, id, mcu_fv, fv, f_size, ret = -1;
    FILE *fp;
    u_char *buf;

    if (argc != 2) {
        printf("Usage: %s <binary file>\n", argv[0]);
        return 0;
    }

    fp = fopen(argv[1], "rb");
    if (fp == NULL) {
        perror("Error opening file!");
        return -1;
    }

    f_size = read_binary_file(fp, &buf);
    if (f_size < 0) {
        printf("Error reading binary file\n");
        return -1;
    }

    printf("update data size %d bytes\n", f_size);

    tty = open_serial_port("/dev/ttyS3", B115200);
    if (tty < 0) {
        printf("Can't open tty device\n");
        return -1;
    }

    if (init_io() < 0) {
        printf("init_io() failed\n");
        return -1;
    }

    enter_boot_mode();

    /*Check whether the MCU has entered boot mode*/
    if (check_mcu_status() != 0) {
        printf("MCU can't enter boot mode\n");
        goto _exit;
    }

    /*get MCU bootloader version*/
    if (get_bootloader_version(&ver) != 0)
        goto _exit;
    printf("MCU bootloader version %d.%d\n", ver/16, ver%16);

    /*get MCU chip id*/
    if (get_chip_id(&id) != 0)
        goto _exit;
    printf("MCU chip id 0x%04x\n", id&0xffff);

    if (id != 0x0410) {
        printf("Chip does not match\n");
        goto _exit;
    }

    if (get_firmware_version(&mcu_fv) < 0) {
        printf("get firmware version failed\n");
        goto _exit;
    }

    printf("MCU firmware version %08x\n", mcu_fv);

    fv = get_update_version(buf);
    if (fv < 0) {
        printf("Invalid update file\n");
        goto _exit;
    }

    printf("Update file firmware version %08x\n", fv);

    if (fv >= mcu_fv) {
        /*erase flash memory*/
        printf("Erase flash memory ... ");
        fflush(stdout);

        if (flash_erase() < 0) {
            printf("failed\n");
            goto _exit;
        }
        printf("done\n");

        printf("Update the firmware to MCU ... ");
        fflush(stdout);

        if (burn_flash(buf, f_size) < 0) {
            printf("failed\n");
            goto _exit;
        }
        printf("done\n");
        ret = 0;
    } else {
        ret = 0;
        printf("It‘s seem that update file is an older version, no need to update\n");
    }

_exit:
    reset_mcu();
    cleanup();

    return ret;
}
