/*
 * **********************************
 * * Program to control ICOM radios *
 * **********************************
 */
#define IBMPC			/* define for little silicon */
/* #define UNIX			/* define for big silicon */

#include <stdio.h>
#include <ctype.h>
#ifdef IBMPC
#include <string.h>
#else
#include <fcntl.h>		/* like it says in the man pages */
#include <sys/types.h>
#include <sys/uio.h>
#endif

/*
 * Parameters
 */
#define TRUE 1
#define FALSE 0

#define PR 0xFE                 /* preamble */
#define TX 0xE0                 /* controller address */
#define FI 0xFD                 /* end of message */

#define ACK 0xFB                /* CN normal reply */
#define NAK 0xFA                /* CN error reply */
#define BMAX 13                 /* packet buffer size */
#define SMAX 50                 /* string buffer size */
#define XFRETRY 100000          /* interface timeout counter */
#define ECRETRY 4               /* bus collision counter */
#define TXRETRY 4               /* retransmission counter */

#ifdef IBMPC
/*
 * Define port and speed
 */
#define PORT 0x03f8             /* port address (COM1) */
/* #define PORT 0x02f8             /* port address (COM2) */
/* #define PORT 0x03e8             /* port address (COM3) */
/* #define PORT 0x02e8             /* port address (COM4) */
/* #define BAUD 384                /* baud rate 300 */
#define BAUD 96                 /* baud rate 1200 */
/* #define BAUD 12                 /* baud rate 9600 */

/*
 * Serial port definitions (8250)
 */
#define THR 0                   /* transmitter holding register */
#define RBR 0                   /* receiver buffer register */
#define DLL 0                   /* divisor latch LSB */
#define DLM 1                   /* divisor latch MSB */
#define LCR 3                   /* line control register */
#define   LCR_8BITS 3           /* 8 bit words */
#define   LCR_DLAB 0x80         /* divisor latch access bit */
#define MCR 4                   /* modem control register */
#define   MCR_DTR 1             /* data terminal ready */
#define   MCR_RTS 2             /* request to send */
#define LSR 5                   /* line status register */
#define   LSR_DR 0x01           /* data ready */
#define   LSR_THRE 0x20         /* transmitter line holding register empty */
#define   LSR_TSRE 0x40         /* transmitter shift register empty */
#else

#endif

/*
 * Control flags
 */
#define f_rcv 0x0001            /* 0: transceiver, 1: receiver */
#define f_vfo 0x0002            /* 0: vfo, 1: no vfo (R7000) */

/*
 * Function declarations
 */
FILE *fopen();

/*
 * Global declarations
 */
int debug = 0;                  /* 0: none, 1: error msgs, 2: packet trace */

/*
 * Radio command prototypes
 */
int s_freqt[] = {0x00,0,0,0,0,0,FI}; /* frequency set (transceive mode) 
*/
int s_modet[] = {0x01,0,0,FI}; /* mode set (transceive mode) */
int s_rband[] = {0x02,FI};      /* band edge read */
int s_rfreq[] = {0x03,FI};      /* frequency read */
int s_rmode[] = {0x04,FI};      /* mode read */
int s_sfreq[] = {0x05,0,0,0,0,0,FI}; /* frequency set */
int s_smode1[] = {0x06,0,FI};   /* mode set */
int s_smode2[] = {0x06,0,0,FI}; /* extended mode set */
int s_svfo[]  = {0x07,FI};      /* vfo set */
int s_chan[]  = {0x08,0,FI};    /* memory channel set */
int s_write[] = {0x09,FI};      /* memory write */
int s_read[]  = {0x0a,FI};      /* memory -> vfo */
int s_clear[] = {0x0b,FI};      /* memory clear */

/*
 * Mode decode
 */
static struct modetable {
	char name [10];         /* mode name */
	int mode;               /* mode code */
	} mode[] = {
	"LSB", 0x0000,          /* lower sideband A3J */
	"USB", 0x0001,          /* upper sideband A3J */
	"AM", 0x0002,           /* AM 6A3 */
	"CW", 0x0003,           /* CW */
	"CWn", 0x8203,          /* CW narrow */
	"RTTY", 0x0004,         /* RTTY */
	"RTTYn", 0x8204,        /* RTTY narrow */
	"FM", 0x0005,           /* FM */
	"FMn", 0x8205,          /* FM narrow (R7000) */
	"SSB", 0x8005,          /* SSB (R7000) */
	"?", 0x00ff,            /* empty channel (R7000) */
	"\0", 0                 /* end of table */
	} ;

/*
 * Radio control structure
 */
static struct icomtable {
	char name[10];          /* radio name */
	int ident;              /* bus address */
	int maxch;              /* max memory channels */
	int flags;              /* type bits */
	int mchan;              /* memory channel */
	int mode;               /* vfo mode */
	long vfo;               /* vfo frequency (MHz) */
	long uband;             /* upper band edge (MHz) */
	long lband;             /* lower band edge (MHz) */
	} icom[] = {
	"761", 0x1E, 32, 0, 0, 0, 0, 0, 0,
	"725", 0x04, 26, 0, 0, 0, 0, 0, 0,
	"735", 0x28, 12, 0, 0, 0, 0, 0, 0,
	"275", 0x10, 99, 0, 0, 0, 0, 0, 0,
	"375", 0x12, 99, 0, 0, 0, 0, 0, 0,
	"475", 0x14, 99, 0, 0, 0, 0, 0, 0,
	"575", 0x16, 99, 0, 0, 0, 0, 0, 0,
	"271", 0x20, 32, 0, 0, 0, 0, 0, 0,
	"471", 0x22, 32, 0, 0, 0, 0, 0, 0,
	"1271", 0x24, 32, 0, 0, 0, 0, 0, 0,
	"751", 0x1C, 32, 0, 0, 0, 0, 0, 0,
	"r71", 0x1A, 32, f_rcv, 0, 0, 0, 0, 0,
	"r7000", 0x08, 99, f_rcv|f_vfo, 0, 0, 0, 0, 0,
	"all", 0x00, 0, 0, 0, 0, 0, 0, 0,
	"\0", 0, 0, 0, 0, 0, 0, 0, 0 /* end of table */
	} ;

int resp[BMAX];                 /* response buffer */
char freq[SMAX];                /* frequency */
int i, temp;                    /* temps */
char fn_com[12] = "COM1";       /* I/O port */
int radio = 0;                  /* structure index */
long temp1;
FILE *fp_in;                    /* input file */
FILE *fp_out;                   /* output file */
FILE *fp_tmp;                   /* temp file */
FILE *fp_icom;			/* serial line */

/*
 * Main program
 */
main() {
	char s1[SMAX], s2[SMAX]; /* parameter strings */
	int par1, par2;         /* integer parameters */
	long freqlong();

/*
 * Initialize
 */
#ifdef IBMPC
	outp(PORT+LCR, LCR_DLAB);  /* set baud */
	outpw(PORT+DLL, BAUD);
	outp(PORT+LCR, LCR_8BITS); /* set 8 bits, no parity */
	outp(PORT+MCR, MCR_DTR+MCR_RTS); /* wake up modem */
#else
	if ((fp_icom = open("/dev/ttyz03", O_RDWR, 0777)) < 0) {
		printf("\n*** Unable to open serial line\n");
		exit(1);
		}
	if (ioctl(fp_icom, TIOCEXCL, (char *)0) < 0) {
		printf("\n*** Unable to set exclusive mode\n");
		exit(1);
		}
#endif
/*
 * Main loop
 */
	fp_in = stdin; fp_out = NULL;
	for (;;) {
		if (fp_in != stdin)
			if (fscanf(fp_in, " %s", s1) == EOF) fp_in = stdin;
		if (fp_in == stdin) {
			printf("icom>"); fscanf(fp_in, " %s", s1);
			}

	/* select radio (name) */

		if (strcmp(s1, "radio") == 0) {
			fscanf(fp_in, " %s", s1);
			for (i = 0; strcmp(s1, icom[i].name) != 0
				&& icom[i].name[0] != '\0'; i++);
			if (icom[i].name[0] == '\0') {
					printf("*** Unknown radio %s\n", s1);
					continue;
					}
			radio = i;
			temp = sndpkt(icom[radio].ident,s_rband,resp);
			if (temp != 12 || resp[0] != s_rband[0]) {
				printf("*** Cannot read band for radio %s\n",
					icom[radio].name);
				continue;
				}
			icom[radio].lband = freqlong(&resp[1]);
			icom[radio].uband = freqlong(&resp[7]);
			if (icom[radio].lband > icom[radio].uband) {
				temp1 = icom[radio].lband;
				icom[radio].lband = icom[radio].uband;
				icom[radio].uband = temp1;
				}
			printf("Band%11.5f MHz -%10.5f MHz\n",
				icom[radio].lband/1e6, icom[radio].uband/1e6);
			continue;
			}

	/* input file (filename) */

		if (strcmp(s1, "input") == 0) {
			fscanf(fp_in, " %s", s1);
			if (fp_in != stdin) {
				fclose(fp_in); fp_in = stdin;
				}
			if (fp_out != NULL) {
				fclose(fp_out); fp_out = NULL;
				}
			if ((fp_tmp = fopen(s1, "r")) != NULL) fp_in = fp_tmp;
			else printf ("*** Input file %s not found\n", s1);
			continue;
			}

	/* output file (filename) */

		if (strcmp(s1, "output") == 0) {
			fscanf(fp_in, " %s", s1);
			if (fp_out != NULL) fclose(fp_out);
			fp_out = fopen (s1, "w");
			continue;
			}

	/* debug switch (magic) */

		if (strcmp(s1, "debug") == 0) {
			fscanf(fp_in, " %i", &debug); continue;
			}

	/* read vfo/memory channel (channel) */

		if (strcmp(s1, "read") == 0) {
			fscanf(fp_in, " %i", &par1);
			if (readch(par1) == 0) display(par1);
			continue;
			}

	/* scan vfo/memory channels (low-channel high-channel)*/

		if (strcmp(s1, "scan") == 0) {
			fscanf(fp_in, " %i %i", &par1, &par2);
			for (i = par1; i <= par2; i++) {
				if (readch(i) != 0) break;
				if (icom[radio].vfo != 0L) display(i);
				}
			continue;
			}

	/* load frequency (frequency) */

		if (strcmp(s1, "freq") == 0) {
			fscanf(fp_in, " %s", s1); sndfreq(s1); continue;
			}

	/* load mode (mode) */

		if (strcmp(s1, "mode") == 0) {
			fscanf(fp_in, " %s", s1); sndmode(s1); continue;
			}

	/* load channel (chan, s1, mode) */

		if (strcmp(s1, "chan") == 0) {
			fscanf(fp_in, " %i %s %s", &par1, s1, s2);
			if (strcmp(s2, "MHz") == 0) fscanf(fp_in, " %s", s2);
			if (setch(par1) != 0) continue;
			if (sndmode(s2) != 0) continue;
			if (sndfreq(s1) != 0) continue;
			temp = sndpkt(icom[radio].ident, s_write, resp);
			if (temp != 1 || resp[0] != ACK)
				printf("*** Cannot write channel for radio %s\n",
					icom[radio].name);
			else display(par1);
			continue;
			}

	/* memory write (no arguments) */

		if (strcmp(s1, "write") == 0) {
			temp = sndpkt(icom[radio].ident, s_write, resp);
			if (temp != 1 || resp[0] != ACK)
				printf("Radio %s can't do that\n",
					icom[radio].name);
			continue;
			}

	/* memory clear (no arguments) */

		if (strcmp(s1, "clear") == 0) {
			temp = sndpkt(icom[radio].ident, s_clear, resp);
			if (temp != 1 || resp[0] != ACK)
				printf("*** Radio %s can't do that\n",
					icom[radio].name);
			continue;
			}

	/* quit (no arguments) */

		if (strcmp(s1, "quit") == 0) exit(0);

	/* none of the above */
		printf("*** Unknown command %s\n", s1);
		}
	}

/*
 * Set channel
 *     c = channel; returns: 0 normal, 1 error
 */
int setch(c) int c; {
	int buf[BMAX], temp;

	temp = 1; buf[0] = ACK;
	if (c == 0) {
		if ((icom[radio].flags & f_vfo) == 0)
			temp = sndpkt(icom[radio].ident, s_svfo, buf);
		}
	else {
		if (c > icom[radio].maxch) {
			printf("*** Undefined channel for radio %s\n",
				icom[radio].name);
			return(1);
			}
		s_chan[1] = c%10+(c/10)*16;
		temp = sndpkt(icom[radio].ident, s_chan, buf);
		}
	if (temp != 1 || buf[0] != ACK) {
		printf("*** Cannot set channel for radio %s\n",
			icom[radio].name);
		return(1);
		}
	return(0);
	}

/*
 * Read channel
 *     c = channel; returns: 0 normal, 1 error
 */
int readch(c) int c; {
	int buf[BMAX], temp;

	if (setch(c) != 0) return 1;
	temp = sndpkt(icom[radio].ident, s_rmode, buf);
	if (temp < 2 || buf[0] != s_rmode[0]) {
		printf("*** Cannot read mode for radio %s\n",
			icom[radio].name);
		return(1);
		}
	icom[radio].mode = buf[1];
	if (temp > 2) icom[radio].mode = icom[radio].mode
		| (buf[2] << 8) | 0x8000;
	temp = sndpkt(icom[radio].ident, s_rfreq, buf);
	if (temp < 2 || buf[0] != s_rfreq[0]) {
		printf("*** Cannot read frequency for radio %s\n",
			icom[radio].name);
		return(1);
		}
	if (temp == 6) icom[radio].vfo = freqlong(&buf[1]);
	else icom[radio].vfo = 0L;
	return(0);
	}

/*
 * Load frequency
 *     s = frequency; returns: 0 normal, 1 error
 */
int sndfreq(s) char s[]; {
	int buf[BMAX], i, k;
	char t[SMAX];
	long temp2;

	k = 0;
	for (i = 0; (t[i] = s[i]) != '\0'; i++) if (s[i] == '.') k = i;
	if (k == 0) k = i;
	for (; i<SMAX; i++) t[i] = '0';
	for (i = k; i<SMAX-1; i++) t[i] = t[i+1];
	for (i = SMAX-1; i>=0; i--) {
		if (i<4) t[i] = '0';
		else t[i] = t[i-4];
		}
	ascfreq(&t[k], &s_sfreq[1]);
	temp2 = freqlong(&s_sfreq[1]);
	if (temp2 < icom[radio].lband || temp2 > icom[radio].uband) {
		printf("*** Frequency outside band for radio %s\n",
			icom[radio].name);
		return(1);
		}
	temp = sndpkt(icom[radio].ident, s_sfreq, buf);
	if (temp != 1 || buf[0] != ACK) {
		printf("*** Invalid frequency for radio %s\n",
			icom[radio].name);
		return(1);
		}
	icom[radio].vfo = freqlong(&s_sfreq[1]);
	return(0);
	}

/*
 * Load mode
 *     s = mode; returns: 0 normal, 1 error
 */
int sndmode(s) char s[]; {
	int buf[BMAX], i;

	for (i = 0; strcmp(mode[i].name, s) != 0; i++)
		if (mode[i].name[0] == '\0') {
			printf("*** Unknown mode %s\n", s);
			return(1);
			}
	if ((mode[i].mode & 0x8000) == 0) {
		s_smode1[1] = mode[i].mode;
		temp = sndpkt(icom[radio].ident, s_smode1, buf);
		}
	else {
		s_smode2[1] = mode[i].mode & 0xff;
		s_smode2[2] = (mode[i].mode >> 8) & 0x7f;
		temp = sndpkt(icom[radio].ident, s_smode2, buf);
		}
	if (temp != 1 || buf[0] != ACK) {
		printf("*** Invalid mode for radio %s\n", icom[radio].name);
		return(1);
		}
	icom[radio].mode = mode[i].mode;
	return(0);
	}

/*
 * Display frequency/mode
 */
int display(c) int c; {
	int i;

	for (i = 0; mode[i].mode != icom[radio].mode
		&& mode[i].name[0] != '\0'; i++);
	printf("Chan%3i%11.5f MHz  %s\n", c, icom[radio].vfo/1e6,
		mode[i].name);
	if (fp_out != NULL)
		fprintf(fp_out, "Chan%3i%11.5f MHz  %s\n",
			c, icom[radio].vfo/1e6, mode[i].name);
	}

/*
 * Conversion routines
 *
 * These routines convert between internal formats and radio format.
 * Memory-channel and frequency data are in 4-bit BCD digits, packed
 * two digits per octet. Memory-channel data consist of two digits.
 * Frequency data consist of ten digits in reverse order, in units of
 * Hz.
 */
/*
 * Convert ASCII string to radio frequency
 *     x = string pointer, y = frequency pointer
 */
int ascfreq(x, y) char *x; int *y; {
	int i, temp;

	y = y + 4;
	for (i = 0; i < 5; i++) {               /* pack and reverse */
		temp = (*x &0x0f)<<4; x++;
		*y = temp | (*x &0x0f); x++; y--;
		}
	}

/*
 * Convert long int to radio frequency
 *     x = long int, y = frequency pointer
 */
int longfreq(x, y) long x; int *y; {
	char s[11];

	sprintf(s, "%lu", x);                   /* convert to ASCII */
	ascfreq(s, y);                          /* convert to radio */
	return(0);
	}

/*
 * Convert radio frequency to long int
 *     x = frequency pointer, y = string pointer
 */
long freqlong(x) int *x; {
	int i; char s[11]; char *y; long atol();

	x = x + 4; y = s;
	for (i = 0; i < 5; i++) {               /* unpack and reverse */
		*y = ((*x >> 4)&0x0f)+'0'; y++;
		*y = (*x & 0x0f)+'0'; y++; x--;
		*y = '\0';                      /* store terminator */
		}
	return(atol(s));
	}

/*
 * Packet routines
 *
 * These routines send a packet and receives the response. If a
 * collision is detected on transmit (by the interface routines) the
 * packet is resent. If a preamble error occurs on receive the response
 * is discarded (including the terminating FI) and the packet is
 * resent. If the maximum number of retries is exceeded or the response
 * frame exceeds the size of the buffer, the program aborts.
 *
 * ICOM frame format
 *
 * Frames begin with a two-octet preamble PR-PR followed be the
 * transceiver address RE, controller address TX, control code CN, a
 * number of data octets DA (depending on command) and the terminator
 * FI. Since the bus is bidirectional, every octet output by the
 * computer is echoed on input. Every valid frame sent by the computer
 * is answered with a frame in the same format, but with the RE and TX
 * fields interchanged. The CN field is set to NAK in case of error and
 * to ACK in case of a valid command that returns no data.
 *
 * +------+------+------+------+------+--//--+------+
 * |      |      |      |      |      |      |      |
 * |  PR  |  PR  |  RE  |  TX  |  CN  |  DA  |  FI  |
 * |      |      |      |      |      |      |      |
 * +------+------+------+------+------+--//--+------+
 */

/*
 * Send packet and receive response
 *     r = radio address, x[] = command data, y[] = response data;
 *     returns octet count
 */
int sndpkt(r, x, y) int r, x[], y[]; {
	int i, j, temp;

	for (i = 0; i < TXRETRY; i++) {
		if (debug > 1) printf("T:");
		if ((temp = sndoctet(PR)) != PR) continue;
		if ((temp = sndoctet(PR)) != PR) continue;
		if ((temp = sndoctet(r)) != r) continue;
		if ((temp = sndoctet(TX)) != TX) continue;
		for (j = 0; (j < BMAX); j++) {  /* send body */
			temp = sndoctet(x[j]);
			if (x[j] == FI) break;
			}
		if (debug > 1) printf("\nR:");
		if (x[0] <= 1) return(0);       /* shortcut for broadcast */
		j = 0;
		do if ((temp = rcvoctet()) == FI) break; /* wait for PR */
			while (temp != PR);
		do if ((temp = rcvoctet()) == FI) break; /* wait for non-PR */
			while (temp == PR);
		if (temp != TX) continue;       /* check header */
		if (temp = (rcvoctet() != r)) continue;
		for (j = 0; (j < BMAX); j++) {  /* receive body */
			y[j] = rcvoctet();
			if (y[j] == FI) {
				if (debug > 1) printf("\n");
				return(j);
				}
			}
		if (j >= BMAX) {                /* check for overflow */
			printf("\n*** Buffer overflow\n");
			exit(1);
			}
		while (temp != FI)              /* discard broken frame */
		temp = rcvoctet();
		}
	printf("\n*** Radio not responding\n");       /* max exceeded */
	exit(1);
	}

/*
 * Interface routines
 *
 * These routines read and write octets on the bus. In case of receive
 * timeout a FI code is returned. In case of output collision (echo
 * does not match octet sent), the remainder of the collision frame
 * (including the trailing FI) is discarded.
 */

#ifdef IBMPC
/*
 * Send octet
 *     x = octet; returns octet transmitted, force FI if error
 */
int sndoctet(x) int x; {
	int i, temp;

	inp(PORT+RBR);                          /* flush spikes */
	while (inp(PORT+LSR)&LSR_THRE == 0);    /* wait for buffer */
	outp(PORT+THR, x);
	if (x == (temp = rcvoctet())) return(temp); /* return echo */
	if (debug > 0) printf("\n*** Bus collision\n", temp);
	while ((temp = rcvoctet()) != FI);      /* discard packet */
	return(temp);
	}

/*
 * Receive octet
 *     returns octet received, force FI if error
 */
int rcvoctet() {
	long i; int temp;

	for (i = 0; i < XFRETRY; i++)           /* wait for buffer */
		if (inp(PORT+LSR)&LSR_DR == 1) {
			temp = inp(PORT+RBR);
			if (debug > 1) printf("%5x", temp);
			return(temp);           /* return octet */
			}
	if (debug > 0) printf("\n*** Interface timeout\n"); /* timeout */
	return(FI);                             /* force FI */
	}
#else
/*
 * Send octet
 *     x = octet; returns octet transmitted, force FI if error
 */
int sndoctet(x) int x; {
	int temp;

	write(fp_icom, x, 1);
	if (x == (temp = rcvoctet())) return(temp); /* return echo */
	if (debug > 0) printf("\n*** Bus collision\n", temp);
	while ((temp = rcvoctet()) != FI);      /* discard packet */
	return(temp);
	}

/*
 * Receive octet
 *     returns octet received, force FI if error
 */
int rcvoctet() {
	int temp;

	read(fp_icom, &temp, 1);
	if (debug > 1) printf("%5x", temp);
	return(temp);           		/* return octet */
	}
#endif

/* end program */
                                                                                                                                                                               