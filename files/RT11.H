/*	RT-11 INTERFACE HEADER FILE
 *	copyright (c) 1978 by Whitesmiths, Ltd.
 */

/*	the Whitesmiths RT-11 control block
 */
#define RCB	struct rcb

struct rcb
	{
	TINY rlun, flags;
	BYTES bleft;
	TEXT *bnext, *pbuf;
	LONG lseek, lact, lend;
	};

#define CTRLZ	032
#define WOPEN	1
#define WTTY	2
#define WCR		4
#define WDIRT	16
#define NFILES	10

/* RT-11 innards
 */
struct {
	COUNT _int;
	};
struct {
	TINY _char;
	};

#define JSW		(044->_int)
#define EMTERR	(052->_char)
#define END		(050->_int)
#define FACRE	1
#define FADEL	2
#define FAOPN	0
#define MAXCMD	183
#define STARTCMD 0510
#define SYSCLOS 03000
#define SYSDEL  00000
#define SYSDSTAT 0342
#define SYSENT  01000
#define SYSOPEN 00400
#define SYSREAD 04000
#define SYSWRIT 04400
                                                                                                                                                                                                                                              