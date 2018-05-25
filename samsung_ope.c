#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <ncurses.h>

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define DC1	0x11	/* Device Control 1 */
#define ACK 	0xb0
#define NAK	0xc0

#define CMD_LCD	0xa1	/* LCD text command */
#define CMD_LED	0xac	/* LED control command */
#define CMD_RST 0xb1	/* reset(?) command */

enum { PAIR_OFF = 1, PAIR_RED, PAIR_GREEN, PAIR_BLUE, PAIR_YELLOW };

void write_byte(int fd, unsigned char byte) {
	write(fd, &byte, 1);
}

void read_bytes(int fd, unsigned char *buf, int count) {
	int n;
	unsigned char *bufptr = buf;

	while ((n = read(fd, bufptr, buf + count - bufptr)) > 0) {
		bufptr += n;
		if (bufptr - buf > count)
			break;
	}
}

void set_leds(unsigned char b1, unsigned char b2) {
	mvprintw(5, 0, "LED bytes: %02hhx %02hhx", b1, b2);
	attron(A_BOLD);

	attron(COLOR_PAIR((b2 & 0x01) == 0 ? PAIR_BLUE : PAIR_OFF));
	mvprintw(4, 0, "STANDBY ");

	/* there are two (dual-color) STATUS LEDs - marked LED4 LEFT and LED5 RIGHT */
	if ((b1 & 0x06) == 0)
		attron(COLOR_PAIR(PAIR_YELLOW));
	else if ((b1 & 0x02) == 0)
		attron(COLOR_PAIR(PAIR_GREEN));
	else if ((b1 & 0x04) == 0)
		attron(COLOR_PAIR(PAIR_RED));
	else
		attron(COLOR_PAIR(PAIR_OFF));
	printw("STA");
	if ((b1 & 0x18) == 0)
		attron(COLOR_PAIR(PAIR_YELLOW));
	else if ((b1 & 0x08) == 0)
		attron(COLOR_PAIR(PAIR_GREEN));
	else if ((b1 & 0x10) == 0)
		attron(COLOR_PAIR(PAIR_RED));
	else
		attron(COLOR_PAIR(PAIR_OFF));
	printw("TUS ");

	attron(COLOR_PAIR((b1 & 0x20) == 0 ? PAIR_GREEN : PAIR_OFF));
	printw("ECO ");

	attron(COLOR_PAIR((b1 & 0x40) == 0 ? PAIR_BLUE : PAIR_OFF));
	printw("WPS ");
}

unsigned char checksum(unsigned char *buf, int len) {
	unsigned char sum = 0;
	int i;

	for (i = 0; i < len; i++)
		sum ^= buf[i];

	return sum;
}

void usage() {
	printf("Samsung printer operator panel emulator\n");
	printf("Copyright (c) 2018 Ondrej Zary\n\n");
	printf("Usage: samsung_ope <device>\n");
	printf("  e.g. samsung_ope /dev/ttyS1\n\n");
}

int main(int argc, char *argv[]) {
	unsigned char buf[258]; /* 3 bytes (header) + max. 255 data bytes */
	struct termios options;
	bool key_pressed = false;

	if (argc < 2) {
		usage();
		return 1;
	}

	int fd = open(argv[1], O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror("Error opening device");
		return 2;
	}

	fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);		/* get the current options */
	cfsetispeed(&options, B9600);		/* set 9600 bps */
	cfsetospeed(&options, B9600);
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;			/* 8 data bits */
	options.c_cflag &= ~PARENB;		/* no parity */
	options.c_cflag &= ~CSTOPB;		/* 1 stop bit */
	options.c_cflag &= ~CRTSCTS;		/* no RTS/CTS */
	options.c_cflag |= (CLOCAL | CREAD); 	/* local link, enable receiver */
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* no canonical, echo and signals */
	options.c_oflag &= ~OPOST;		/* no postprocessing (RAW mode) */
	options.c_iflag &= ~(INPCK | INLCR | ICRNL | IXON | IXOFF | IXANY);	/* disable input parity check, CR/LF conversions and software flow control */
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 5;		/* 0.5 second timeout */
	tcsetattr(fd, TCSANOW, &options);	/* set the options */

	WINDOW *frame_win, *lcd_win, *debug_win;
	initscr();
	start_color();
	init_pair(PAIR_OFF, COLOR_BLACK, COLOR_BLACK);
	init_pair(PAIR_RED, COLOR_RED, COLOR_BLACK);
	init_pair(PAIR_GREEN, COLOR_GREEN, COLOR_BLACK);
	init_pair(PAIR_BLUE, COLOR_BLUE, COLOR_BLACK);
	init_pair(PAIR_YELLOW, COLOR_YELLOW, COLOR_BLACK);
	timeout(0);
	clear();
	curs_set(0);
	noecho();
	cbreak();
	
	frame_win = derwin(stdscr, 4, 18, 0, 0);
	box(frame_win, 0, 0);
	lcd_win = derwin(frame_win, 2, 16, 1, 1);
	debug_win = derwin(stdscr, MIN(20, LINES-6), COLS, MAX(LINES - 20, 6), 0);
	scrollok(debug_win, 1);
	keypad(stdscr, true);
	refresh();
	wrefresh(frame_win);

	while (1) {
		unsigned char sum;
		int n = read(fd, &buf[0], 1);
		if (n > 0) {
			wprintw(debug_win, "%02hhx ", buf[0]);
			if (buf[0] != DC1)
				continue;
			read_bytes(fd, &buf[1], 2);
			read_bytes(fd, &buf[3], buf[2]);
			for (int i = 0; i < buf[2] + 2; i++)
				wprintw(debug_win, "%02hhx ", buf[i + 1]);

			sum = checksum(&buf[1], buf[2] + 2);
			if (sum) {
				write_byte(fd, NAK);
				wprintw(debug_win, "nonzero checksum: 0x%02x\n", sum);
				wrefresh(debug_win);
				continue;
			}
			wprintw(debug_win, "\n");
			wrefresh(debug_win);

			if (buf[0] == DC1) {
				char lcd_text[33];
				switch (buf[1]) {
				case CMD_LCD:
					strncpy(lcd_text, (char *) &buf[3], sizeof(lcd_text) - 1);
					lcd_text[32] = '\0';
					for (unsigned int i = 0; i < strlen(lcd_text); i++) {
						if (lcd_text[i] == 0x12)
							lcd_text[i] = '<';
						else if (lcd_text[i] == 0x13)
							lcd_text[i] = '>';
					}
					mvwprintw(lcd_win, 0, 0, "%s", lcd_text);
					wrefresh(lcd_win);
					break;
				case CMD_LED:
					set_leds(buf[3], buf[4]);
					break;
				case CMD_RST:
					set_leds(0, 0);
					break;
				}
			}
		}
		if (key_pressed) {
			write_byte(fd, 0xaf);	/* key release */
			key_pressed = false;
		} else {
			switch (getch()) {
			case '\n':	write_byte(fd, 'u'); key_pressed = true; break; /* OK */
			case KEY_LEFT:	write_byte(fd, 'G'); key_pressed = true; break; /* < */
			case KEY_RIGHT:	write_byte(fd, 'H'); key_pressed = true; break; /* > */
			case KEY_UP:	write_byte(fd, 'E'); key_pressed = true; break; /* MENU */
			case KEY_DOWN:	write_byte(fd, 'F'); key_pressed = true; break; /* BACK */
			case 'x':	write_byte(fd, 'x'); key_pressed = true; break; /* CANCEL */
			case 'e':	write_byte(fd, 'U'); key_pressed = true; break; /* ECO */
			case 'w':	write_byte(fd, 'V'); key_pressed = true; break; /* WPS */
			}
		}
		write_byte(fd, ACK);
	}

	return 0;
}
