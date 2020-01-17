/* #define	NEW_ASTERISK */

/*
 * Copyright (C) 2010, Bogdan Diaconescu
 *
 * Bogdan Diaconescu, YO3IIU <yo3iiu@y3iiu.ro>
 * Based upon work of:
 * Jim Dixon, WB6NIL <jim@lambdatel.com>
 * Mark Spencer <markster@digium.com> and Luigi Rizzo
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2. See the LICENSE file
 * at the top of the source tree.
 */

/*! \file
 *
 * \brief Simple Channel driver for ALSA interface with serial controls
 *
 * \author Bogdan Diaconescu, YO3IIU  <yo3iiu@yo3iiu.ro>
 *
 * \ingroup channel_drivers
 */

/*** MODULEINFO
        <defaultenabled>yes</defaultenabled> 	 	 
 ***/

#include "asterisk.h"

ASTERISK_FILE_VERSION(__FILE__, "$Revision$")

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
//#include <sys/io.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <errno.h>
#include <alsa/asoundlib.h>
#include <math.h>

#define DEBUG_CAPTURES	 		1

#define RX_CAP_RAW_FILE			"/tmp/rx_cap_in.pcm"
#define TX_CAP_RAW_FILE			"/tmp/tx_cap_in.pcm"

#define	MIXER_PARAM_MIC_PLAYBACK_SW 	"Mic Playback Switch"
#define MIXER_PARAM_MIC_PLAYBACK_VOL 	"Mic Playback Volume"
#define	MIXER_PARAM_MIC_CAPTURE_SW 	"Mic Capture Switch"
#define	MIXER_PARAM_MIC_CAPTURE_VOL 	"Mic Capture Volume"
#define	MIXER_PARAM_MIC_BOOST 		"Auto Gain Control"
#define	MIXER_PARAM_SPKR_PLAYBACK_SW 	"Speaker Playback Switch"
#define	MIXER_PARAM_SPKR_PLAYBACK_VOL 	"Speaker Playback Volume"

#define	DELIMCHR ','
#define	QUOTECHR 34

#define	READERR_THRESHOLD 50

#ifdef __linux
#include <linux/soundcard.h>
#elif defined(__FreeBSD__)
#include <sys/soundcard.h>
#else
#include <soundcard.h>
#endif

#include "asterisk/lock.h"
#include "asterisk/frame.h"
#include "asterisk/logger.h"
#include "asterisk/callerid.h"
#include "asterisk/channel.h"
#include "asterisk/module.h"
#include "asterisk/options.h"
#include "asterisk/pbx.h"
#include "asterisk/config.h"
#include "asterisk/cli.h"
#include "asterisk/utils.h"
#include "asterisk/causes.h"
#include "asterisk/endian.h"
#include "asterisk/stringfields.h"
#include "asterisk/abstract_jb.h"
#include "asterisk/musiconhold.h"
#include "asterisk/dsp.h"

#ifndef	NEW_ASTERISK

/* ringtones we use */
#include "busy.h"
#include "ringtone.h"
#include "ring10.h"
#include "answer.h"

#endif

/* ALSA and serial stuff */
#define ALSA_INDEV		"default"
#define ALSA_OUTDEV		"default"
#define DESIRED_RATE		8000
#define SERIAL_DEV		"/dev/ttyS0"

/* Lets use 160 sample frames, just like GSM.  */
#define FRAME_SIZE 160
#define PERIOD_FRAMES 80		/* 80 Frames, at 2 bytes each */

#if __BYTE_ORDER == __LITTLE_ENDIAN
static snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
#else
static snd_pcm_format_t format = SND_PCM_FORMAT_S16_BE;
#endif

AST_MUTEX_DEFINE_STATIC(alsalock);

static int alsaradio_debug = 0;

/* Number of buffers...  Each is FRAMESIZE/8 ms long.  For example
   with 160 sample frames, and a buffer size of 3, we have a 60ms buffer, 
   usually plenty. */

#define MAX_BUFFER_SIZE 100

int sim_cor = 0;		/* used to simulate COR active */

/* ALSA and serial stuff end */

#define O_CLOSE	0x444

#define	NTAPS 31

/*! Global jitterbuffer configuration - by default, jb is disabled */
static struct ast_jb_conf default_jbconf =
{
	.flags = 0,
	.max_size = -1,
	.resync_threshold = -1,
	.impl = "",
};
static struct ast_jb_conf global_jbconf;

/*
 * alsaradio.conf parameters are
START_CONFIG

[general]
	; General config options which propigate to all devices, with
	; default values shown. You may have as many devices as the
	; system will allow. You must use one section per device, with
	; [alsaradio] generally (although its up to you) being the first device.
	;
	;
	; debug = 0x0			; misc debug flags, default is 0

	; Set the device to use for I/O
	; devicenum = 0

	; rxboost = 0          		; no rx gain boost
	; carrierfrom = serialdsr    	; no,serialdsr,serialdsrinvert
	; ctcssfrom = serialcts 	; no,serialcts,serialctsinvert
					; Serial port signals: PTT=RTS,COR=DSR,CTCSS=CTS
					; DTR=1 always
	; invertptt = 0

        ; duplex = 1			; duplex mode    

	; rxondelay = 0		  	; number of 20ms intervals to hold off receiver turn-on indication

	;------------------------------ JITTER BUFFER CONFIGURATION --------------------------
	; jbenable = yes          ; Enables the use of a jitterbuffer on the receiving side of an
                                  ; alsaradio channel. Defaults to "no". An enabled jitterbuffer will
                                  ; be used only if the sending side can create and the receiving
                                  ; side can not accept jitter. The alsaradio channel can't accept jitter,
                                  ; thus an enabled jitterbuffer on the receive alsaradio side will always
                                  ; be used if the sending side can create jitter.

	; jbmaxsize = 200         ; Max length of the jitterbuffer in milliseconds.

	; jbresyncthreshold = 1000    ; Jump in the frame timestamps over which the jitterbuffer is
                                  ; resynchronized. Useful to improve the quality of the voice, with
                                  ; big jumps in/broken timestamps, usualy sent from exotic devices
                                  ; and programs. Defaults to 1000.

	; jbimpl = fixed              ; Jitterbuffer implementation, used on the receiving side of an alsaradio
                                  ; channel. Two implementations are currenlty available - "fixed"
                                  ; (with size always equals to jbmax-size) and "adaptive" (with
                                  ; variable size, actually the new jb of IAX2). Defaults to fixed.

	; jblog = no              ; Enables jitterbuffer frame logging. Defaults to "no".
	;-----------------------------------------------------------------------------------

[aslsaradio0]

; First channel unique config

input_device=default            ; ALSA input channel
output_device=default           ; ALSA output channel
serial_device=/dev/ttyS0        ; Serial port for control
serial_disable=0		; Disable/ Enable serial port use


[alsaradio1]

; Second channel config

END_CONFIG

 */

/*
 * Helper macros to parse config arguments. They will go in a common
 * header file if their usage is globally accepted. In the meantime,
 * we define them here. Typical usage is as below.
 * Remember to open a block right before M_START (as it declares
 * some variables) and use the M_* macros WITHOUT A SEMICOLON:
 *
 *	{
 *		M_START(v->name, v->value) 
 *
 *		M_BOOL("dothis", x->flag1)
 *		M_STR("name", x->somestring)
 *		M_F("bar", some_c_code)
 *		M_END(some_final_statement)
 *		... other code in the block
 *	}
 *
 * XXX NOTE these macros should NOT be replicated in other parts of asterisk. 
 * Likely we will come up with a better way of doing config file parsing.
 */
#define M_START(var, val) \
        char *__s = var; char *__val = val;
#define M_END(x)   x;
#define M_F(tag, f)			if (!strcasecmp((__s), tag)) { f; } else
#define M_BOOL(tag, dst)	M_F(tag, (dst) = ast_true(__val) )
#define M_UINT(tag, dst)	M_F(tag, (dst) = strtoul(__val, NULL, 0) )
#define M_STR(tag, dst)		M_F(tag, ast_copy_string(dst, __val, sizeof(dst)))

/*
 * The following parameters are used in the driver:
 *
 *  FRAME_SIZE	the size of an audio frame, in samples.
 *		160 is used almost universally, so you should not change it.
 *
 *  FRAGS	the argument for the SETFRAGMENT ioctl.
 *		Overridden by the 'frags' parameter in alsaradio.conf
 *
 *		Bits 0-7 are the base-2 log of the device's block size,
 *		bits 16-31 are the number of blocks in the driver's queue.
 *		There are a lot of differences in the way this parameter
 *		is supported by different drivers, so you may need to
 *		experiment a bit with the value.
 *		A good default for linux is 30 blocks of 64 bytes, which
 *		results in 6 frames of 320 bytes (160 samples).
 *		FreeBSD works decently with blocks of 256 or 512 bytes,
 *		leaving the number unspecified.
 *		Note that this only refers to the device buffer size,
 *		this module will then try to keep the lenght of audio
 *		buffered within small constraints.
 *
 *  QUEUE_SIZE	The max number of blocks actually allowed in the device
 *		driver's buffer, irrespective of the available number.
 *		Overridden by the 'queuesize' parameter in alsaradio.conf
 *
 *		Should be >=2, and at most as large as the hw queue above
 *		(otherwise it will never be full).
 */

#define FRAME_SIZE	160
#define	QUEUE_SIZE	5

#if defined(__FreeBSD__)
#define	FRAGS	0x8
#else
#define	FRAGS	( ( (5 * 6) << 16 ) | 0xa )
#endif

/*
 * XXX text message sizes are probably 256 chars, but i am
 * not sure if there is a suitable definition anywhere.
 */
#define TEXT_SIZE	256

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

static char *config = "alsaradio.conf";			/* default config file */
static char *config1 = "alsaradio_tune_%s.conf";	/* tune config file */

static FILE *frxcapraw = NULL;
static FILE *ftxcapraw = NULL;

enum {CD_IGNORE,SERIAL_DSR,SERIAL_DSR_INVERT};
enum {SD_IGNORE,SERIAL_CTS,SERIAL_CTS_INVERT};		// no,external,externalinvert,software

/*	DECLARE STRUCTURES */

/*
 * Each sound is made of 'datalen' samples of sound, repeated as needed to
 * generate 'samplen' samples of data, then followed by 'silencelen' samples
 * of silence. The loop is repeated if 'repeat' is set.
 */
struct sound {
	int ind;
	char *desc;
	short *data;
	int datalen;
	int samplen;
	int silencelen;
	int repeat;
};

#ifndef	NEW_ASTERISK

static struct sound sounds[] = {
	{ AST_CONTROL_RINGING, "RINGING", ringtone, sizeof(ringtone)/2, 16000, 32000, 1 },
	{ AST_CONTROL_BUSY, "BUSY", busy, sizeof(busy)/2, 4000, 4000, 1 },
	{ AST_CONTROL_CONGESTION, "CONGESTION", busy, sizeof(busy)/2, 2000, 2000, 1 },
	{ AST_CONTROL_RING, "RING10", ring10, sizeof(ring10)/2, 16000, 32000, 1 },
	{ AST_CONTROL_ANSWER, "ANSWER", answer, sizeof(answer)/2, 2200, 0, 0 },
	{ -1, NULL, 0, 0, 0, 0 },	/* end marker */
};

#endif

/*
 * descriptor for one of our channels.
 * There is one used for 'default' values (from the [general] entry in
 * the configuration file), and then one instance for each device
 * (the default is cloned from [general], others are only created
 * if the relevant section exists).
 */
struct chan_alsaradio_pvt {
	struct chan_alsaradio_pvt *next;

	char *name;
#ifndef	NEW_ASTERISK
	/*
	 * cursound indicates which in struct sound we play. -1 means nothing,
	 * any other value is a valid sound, in which case sampsent indicates
	 * the next sample to send in [0..samplen + silencelen]
	 * nosound is set to disable the audio data from the channel
	 * (so we can play the tones etc.).
	 */
	int sndcmd[2];				/* Sound command pipe */
	int cursound;				/* index of sound to send */
	int sampsent;				/* # of sound samples sent  */
	int nosound;				/* set to block audio from the PBX */
#endif

	int devtype;				/* actual type of device */
	int pttkick[2];
	int total_blocks;			/* total blocks in the output device */
	int sounddev;
	enum { M_UNSET, M_FULL, M_READ, M_WRITE } duplex;
	short cdMethod;
	int autoanswer;
	int autohangup;
	int hookstate;
	int usedtmf;
	unsigned int queuesize;		/* max fragments in queue */
	unsigned int frags;			/* parameter for SETFRAGMENT */

	int warned;					/* various flags used for warnings */
#define WARN_used_blocks	1
#define WARN_speed		2
#define WARN_frag		4
	int w_errors;				/* overfull in the write path */
	struct timeval lastopen;

	int overridecontext;
	int mute;

	/* boost support. BOOST_SCALE * 10 ^(BOOST_MAX/20) must
	 * be representable in 16 bits to avoid overflows.
	 */
#define	BOOST_SCALE	(1<<9)
#define	BOOST_MAX	40			/* slightly less than 7 bits */
	int boost;					/* input boost, scaled by BOOST_SCALE */
	char devicenum;
	char devstr[128];
	int spkrmax;
	int micmax;

#ifndef	NEW_ASTERISK
	pthread_t sthread;
#endif
	pthread_t serthread;

	int stopser;
	int stopwrite;

	struct ast_channel *owner;
	char ext[AST_MAX_EXTENSION];
	char ctx[AST_MAX_CONTEXT];
	char language[MAX_LANGUAGE];
	char cid_name[256];			/*XXX */
	char cid_num[256];			/*XXX */
	char mohinterpret[MAX_MUSICCLASS];

	/* buffers used in alsaradio_write, 2 per int */
	char alsaradio_write_buf[FRAME_SIZE * 2];    

	int alsaradio_write_dst;
	/* buffers used in alsaradio_read - AST_FRIENDLY_OFFSET space for headers
	 * plus enough room for a full frame
	 */
	char alsaradio_read_buf[FRAME_SIZE * 4 * 6];
	char alsaradio_read_frame_buf[FRAME_SIZE * 2 + AST_FRIENDLY_OFFSET];
#ifdef 	OLD_ASTERISK
	AST_LIST_HEAD(, ast_frame) txq;
#else
	AST_LIST_HEAD_NOLOCK(, ast_frame) txq;
#endif
	ast_mutex_t  txqlock;
	int readpos;				/* read position above */
	struct ast_frame read_f;	/* returned by alsaradio_read */

	char 	debuglevel;
	char 	radioduplex;			// 

	int  	tracetype;
	int     tracelevel;

	char lastrx;
	char rxsersq;
	char rxserctcss;

	char rxkeyed;	  			// indicates rx signal present

	char lasttx;
	char txkeyed;				// tx key request from upper layers 
	char txtestkey;

	time_t lastsertime;
	struct ast_dsp *dsp;

	short	flpt[NTAPS + 1];
	short	flpr[NTAPS + 1];

	char    rxcpusaver;
	char    txcpusaver;

	char 	rxcdtype;
	char 	rxsdtype;

	char	invertptt;

	int	rxondelay;
	int	rxoncnt;

	int	rxboostset;
	int	rxmixerset;
	int 	txmixaset;
	int 	txmixbset;

	struct {
	    unsigned rxcapraw:1;
	    unsigned txcapraw:1;
	}b;
	int readerrs;

	/* ALSA stuff */
	char silencesuppression;
	int  silencethreshold;
	char indevname[256];
	int indev;
	snd_pcm_t *inhandle;
	char outdevname[256];
	int outdev;
	snd_pcm_t *outhandle;

	/* serial stuff */
	char serdevname[256];
	int serdisable;
	int serdev;

	struct timeval tv;
	struct timeval tv2;
};

static struct chan_alsaradio_pvt alsaradio_default = {
#ifndef	NEW_ASTERISK
	.cursound = -1,
#endif
	.sounddev = -1,
	.duplex = 1,
	.autoanswer = 1,
	.autohangup = 1,
	.queuesize = QUEUE_SIZE,
	.frags = FRAGS,
	.ext = "s",
	.ctx = "default",
	.readpos = 0,	/* start here on reads */
	.lastopen = { 0, 0 },
	.boost = BOOST_SCALE,
	.usedtmf = 1,
	.rxondelay = 0,
	/* ALSA stuff */
	.silencesuppression = 0,
	.silencethreshold = 1000,
	.indevname = ALSA_INDEV,
	.indev = -1,
	.outdevname = ALSA_OUTDEV,
	.outdev = -1,
	/* serial stuff */
	.serdevname = SERIAL_DEV,
	.serdisable = 0,
	.serdev = -1
};

/*	DECLARE FUNCTION PROTOTYPES	*/

static void mixer_write(struct chan_alsaradio_pvt *o);
static void tune_write(struct chan_alsaradio_pvt *o);

static char *alsaradio_active;	 /* the active device */

static struct ast_channel *alsaradio_request(const char *type, int format, void *data
, int *cause);
static int alsaradio_digit_begin(struct ast_channel *c, char digit);
static int alsaradio_digit_end(struct ast_channel *c, char digit, unsigned int duration);
static int alsaradio_text(struct ast_channel *c, const char *text);
static int alsaradio_hangup(struct ast_channel *c);
static int alsaradio_answer(struct ast_channel *c);
static struct ast_frame *alsaradio_read(struct ast_channel *chan);
static int alsaradio_call(struct ast_channel *c, char *dest, int timeout);
static int alsaradio_write(struct ast_channel *chan, struct ast_frame *f);
static int alsaradio_indicate(struct ast_channel *chan, int cond, const void *data, size_t datalen);
static int alsaradio_fixup(struct ast_channel *oldchan, struct ast_channel *newchan);
static int alsaradio_setoption(struct ast_channel *chan, int option, void *data, int datalen);

static int setformat(struct chan_alsaradio_pvt *o, int mode);

static snd_pcm_t *alsa_card_init(struct chan_alsaradio_pvt *o, char *dev, snd_pcm_stream_t stream);
static void alsa_card_uninit(struct chan_alsaradio_pvt *o);
static int alsa_write(struct chan_alsaradio_pvt *o, struct ast_frame *f);
static struct ast_frame *alsa_read(struct chan_alsaradio_pvt *o);

static int serial_init(struct chan_alsaradio_pvt *o);
static void serial_uninit(struct chan_alsaradio_pvt *o);
static int serial_getcor(struct chan_alsaradio_pvt *o);
static int serial_getctcss(struct chan_alsaradio_pvt *o);
static int serial_pttkey(struct chan_alsaradio_pvt *o);
static int serial_pttunkey(struct chan_alsaradio_pvt *o);

static const struct ast_channel_tech alsaradio_tech = {
	.type = "alsaradio",
	.description = "alsaradio channel driver",
	.capabilities = AST_FORMAT_SLINEAR,
	.requester = alsaradio_request,
	.send_digit_begin = alsaradio_digit_begin,
	.send_digit_end = alsaradio_digit_end,
	.send_text = alsaradio_text,
	.hangup = alsaradio_hangup,
	.answer = alsaradio_answer,
	.read = alsaradio_read,
	.call = alsaradio_call,
	.write = alsaradio_write,
	.indicate = alsaradio_indicate,
	.fixup = alsaradio_fixup,
	.setoption = alsaradio_setoption,
};

/* FIR Low pass filter, 2900 Hz passband with 0.5 db ripple, 6300 Hz stopband at 60db */

static short lpass(short input,short *z)
{
    int i;
    int accum;
    
    static short h[NTAPS] = {103,136,148,74,-113,-395,-694,
	-881,-801,-331,573,1836,3265,4589,5525,5864,5525,
	4589,3265,1836,573,-331,-801,-881,-694,-395, -113,
	74,148,136,103} ;

    /* store input at the beginning of the delay line */
    z[0] = input;

    /* calc FIR and shift data */
    accum = h[NTAPS - 1] * z[NTAPS - 1];
    for (i = NTAPS - 2; i >= 0; i--) {
        accum += h[i] * z[i];
        z[i + 1] = z[i];
    }

    return(accum >> 15);
}

/* lround for uClibc
 *
 * wrapper for lround(x)
 */
long lround(double x)
{
    return (long) ((x - ((long)x) >= 0.5f) ? (((long)x) + 1) : ((long)x));
}

/* Call with:  devnum: alsa major device number, param: ascii Formal
Parameter Name, val1, first or only value, val2 second value, or 0 
if only 1 value. Values: 0-99 (percent) or 0-1 for baboon.

Note: must add -lasound to end of linkage */

static int setamixer(int devnum,char *param, int v1, int v2)
{
	int	type;
	char	str[100];
	snd_hctl_t *hctl;
	snd_ctl_elem_id_t *id;
	snd_ctl_elem_value_t *control;
	snd_hctl_elem_t *elem;
	snd_ctl_elem_info_t *info;

	sprintf(str,"hw:%d",devnum);
	if (snd_hctl_open(&hctl, str, 0)) return(-1);
	snd_hctl_load(hctl);
	snd_ctl_elem_id_alloca(&id);
	snd_ctl_elem_id_set_interface(id, SND_CTL_ELEM_IFACE_MIXER);
	snd_ctl_elem_id_set_name(id, param);  
	elem = snd_hctl_find_elem(hctl, id);
	if (!elem)
	{
		snd_hctl_close(hctl);
		return(-1);
	}
	snd_ctl_elem_info_alloca(&info);
	snd_hctl_elem_info(elem,info);
	type = snd_ctl_elem_info_get_type(info);
	snd_ctl_elem_value_alloca(&control);
	snd_ctl_elem_value_set_id(control, id);    
	switch(type)
	{
	    case SND_CTL_ELEM_TYPE_INTEGER:
		snd_ctl_elem_value_set_integer(control, 0, v1);
		if (v2 > 0) snd_ctl_elem_value_set_integer(control, 1, v2);
		break;
	    case SND_CTL_ELEM_TYPE_BOOLEAN:
		snd_ctl_elem_value_set_integer(control, 0, (v1 != 0));
		break;
	}
	if (snd_hctl_elem_write(elem, control))
	{
		snd_hctl_close(hctl);
		return(-1);
	}
	snd_hctl_close(hctl);
	return(0);
}

/*
*/
static void kickptt(struct chan_alsaradio_pvt *o)
{
	char c = 0;
	if (!o) return;
	if (!o->pttkick) return;
	write(o->pttkick[1],&c,1);
}

/*
 * returns a pointer to the descriptor with the given name
 */
static struct chan_alsaradio_pvt *find_desc(char *dev)
{
	struct chan_alsaradio_pvt *o = NULL;

	if (!dev)
		ast_log(LOG_WARNING, "null dev\n");

	for (o = alsaradio_default.next; o && o->name && dev && strcmp(o->name, dev) != 0; o = o->next);
	if (!o)
	{
		ast_log(LOG_WARNING, "could not find <%s>\n", dev ? dev : "--no-device--");
		pthread_exit(NULL);
	}

	return o;
}

/*
*/
static void *serthread(void *arg)
{
	unsigned char keyed,ctcssed,txreq;
	char lastrx, fname[200];
	int res;
	struct chan_alsaradio_pvt *o = (struct chan_alsaradio_pvt *) arg;
	struct timeval to;
	struct ast_config *cfg1;
	struct ast_variable *v;
	fd_set rfds;

#ifdef	NEW_ASTERISK
	struct ast_flags zeroflag = {0};
#endif

        while(!o->stopser)
        {
                time(&o->lastsertime);

		//o->micmax = amixer_max(o->devicenum,MIXER_PARAM_MIC_CAPTURE_VOL);
		//o->spkrmax = amixer_max(o->devicenum,MIXER_PARAM_SPKR_PLAYBACK_VOL);

		if (pipe(o->pttkick) == -1)
		{
		    ast_log(LOG_ERROR,"Not able to create pipe\n");
			pthread_exit(NULL);
		}
		ast_log(LOG_NOTICE, "serthread: Starting normally on %s!!\n",o->name);
		lastrx = 0;
                if (option_verbose > 1)
                       ast_verbose(VERBOSE_PREFIX_2 "Set device %s to %s\n",o->devstr,o->name);
		mixer_write(o);

		snprintf(fname,sizeof(fname) - 1,config1,o->name);
#ifdef	NEW_ASTERISK
		cfg1 = ast_config_load(fname,zeroflag);
#else
		cfg1 = ast_config_load(fname);
#endif
		o->rxmixerset = 500;
		o->txmixaset = 500;
		o->txmixbset = 500;
		if (cfg1) {
			for (v = ast_variable_browse(cfg1, o->name); v; v = v->next) {
	
				M_START((char *)v->name, (char *)v->value);
				M_UINT("rxmixerset", o->rxmixerset)
				M_UINT("txmixaset", o->txmixaset)
				M_UINT("txmixbset", o->txmixbset)
				M_END(;
				);
			}
			ast_config_destroy(cfg1);
			ast_log(LOG_WARNING,"Loaded parameters from %s for device %s .\n",fname,o->name);
		} else ast_log(LOG_WARNING,"File %s not found, device %s using default parameters.\n",fname,o->name);

		while(!o->stopser)
		{
			to.tv_sec = 0;
			to.tv_usec = 50000; 

			FD_ZERO(&rfds);
			FD_SET(o->pttkick[0],&rfds);
			/* ast_select emulates linux behaviour in terms of timeout handling */
			res = ast_select(o->pttkick[0] + 1, &rfds, NULL, NULL, &to);
			if (res < 0) {
				ast_log(LOG_WARNING, "select failed: %s\n", strerror(errno));
				usleep(10000);
				continue;
			}
			if (FD_ISSET(o->pttkick[0],&rfds))
			{
				char c;

				read(o->pttkick[0],&c,1);
			}

			keyed = sim_cor || serial_getcor(o);
			if (keyed != o->rxsersq)
			{
				ast_log(LOG_NOTICE, "chan_alsaradio() serthread: update rxsersq = %d\n",keyed);
				o->rxsersq = keyed;
			}

			ctcssed = serial_getctcss(o);
			if (ctcssed != o->rxserctcss)
			{
				ast_log(LOG_NOTICE, "chan_alsaradio() serthread: update rxserctcss = %d\n",ctcssed);
				o->rxserctcss = ctcssed;
			}
			ast_mutex_lock(&o->txqlock);
			txreq = o->txkeyed;			//!(AST_LIST_EMPTY(&o->txq));
			ast_mutex_unlock(&o->txqlock);
			txreq = txreq || o->txtestkey;	
			if (txreq && (!o->lasttx))
			{
				if (o->invertptt) serial_pttunkey(o);
				else serial_pttkey(o);
				ast_log(LOG_NOTICE, "chan_alsaradio() serthread: update PTT = %d\n",txreq);
			}
			else if ((!txreq) && o->lasttx)
			{
				if (o->invertptt) serial_pttkey(o);
				else serial_pttunkey(o);
				ast_log(LOG_NOTICE, "chan_alsaradio() serthread: update PTT = %d\n",txreq);
			}
			o->lasttx = txreq;
			time(&o->lastsertime);
		}
		o->lasttx = 0;
	}
        pthread_exit(NULL);
}

#ifndef	NEW_ASTERISK

/*
 * Handler for 'sound writable' events from the sound thread.
 * Builds a frame from the high level description of the sounds,
 * and passes it to the audio device.
 * The actual sound is made of 1 or more sequences of sound samples
 * (s->datalen, repeated to make s->samplen samples) followed by
 * s->silencelen samples of silence. The position in the sequence is stored
 * in o->sampsent, which goes between 0 .. s->samplen+s->silencelen.
 * In case we fail to write a frame, don't update o->sampsent.
 */
static void send_sound(struct chan_alsaradio_pvt *o)
{
	short myframe[FRAME_SIZE];
	int ofs, l, start;
	int l_sampsent = o->sampsent;
	struct sound *s;
	struct ast_frame f;

	if (o->cursound < 0)		/* no sound to send */
		return;

	s = &sounds[o->cursound];

	for (ofs = 0; ofs < FRAME_SIZE; ofs += l) {
		l = s->samplen - l_sampsent;	/* # of available samples */
		if (l > 0) {
			start = l_sampsent % s->datalen;	/* source offset */
			if (l > FRAME_SIZE - ofs)	/* don't overflow the frame */
				l = FRAME_SIZE - ofs;
			if (l > s->datalen - start)	/* don't overflow the source */
				l = s->datalen - start;
			bcopy(s->data + start, myframe + ofs, l * 2);
			if (0)
				ast_log(LOG_WARNING, "send_sound sound %d/%d of %d into %d\n", l_sampsent, l, s->samplen, ofs);
			l_sampsent += l;
			static const short silence[FRAME_SIZE] = { 0, };

			l += s->silencelen;
			if (l > 0) {
				if (l > FRAME_SIZE - ofs)
					l = FRAME_SIZE - ofs;
				bcopy(silence, myframe + ofs, l * 2);
				l_sampsent += l;
			} else {			/* silence is over, restart sound if loop */
				if (s->repeat == 0) {	/* last block */
					o->cursound = -1;
					o->nosound = 0;	/* allow audio data */
					if (ofs < FRAME_SIZE)	/* pad with silence */
						bcopy(silence, myframe + ofs, (FRAME_SIZE - ofs) * 2);
				}
				l_sampsent = 0;
			}
		}
	}

	f.frametype = AST_FRAME_VOICE;
	f.subclass = AST_FORMAT_SLINEAR;
	f.samples = FRAME_SIZE;
	f.datalen = FRAME_SIZE * 2;
	f.data = myframe;
	f.offset = AST_FRIENDLY_OFFSET;
	f.src = alsaradio_tech.type;
	f.mallocd = 0;

	l = alsa_write(o, &f);
	if (l > 0)
		o->sampsent = l_sampsent;	/* update status */
}

static void *sound_thread(void *arg)
{
	struct chan_alsaradio_pvt *o = (struct chan_alsaradio_pvt *) arg;

	fd_set rfds;
	fd_set wfds;
	int max, res;

	for (;;) {
		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		max = o->sndcmd[0];
		FD_SET(o->sndcmd[0], &rfds);
		if (o->cursound > -1) {
			FD_SET(o->outdev, &wfds);
			if (o->outdev > max)
				max = o->outdev;
		}
#ifdef ALSA_MONITOR
		if (!alsa.owner) {
			FD_SET(o->indev, &rfds);
			if (o->indev > max)
				max = indev;
		}
#endif
		res = ast_select(max + 1, &rfds, &wfds, NULL, NULL);
		if (res < 1) {
			ast_log(LOG_WARNING, "select failed: %s\n", strerror(errno));
			continue;
		}

#ifdef ALSA_MONITOR
		if (FD_ISSET(o->indev, &rfds)) {
			/* Keep the pipe going with read audio */
			snd_pcm_state_t state;
			short buf[FRAME_SIZE];
			int r;

			state = snd_pcm_state(alsa.ocard);
			if (state == SND_PCM_STATE_XRUN) {
				snd_pcm_prepare(alsa.ocard);
			}
			r = snd_pcm_readi(o->inhandle, buf, FRAME_SIZE);
			if (r == -EPIPE) {
				if (alsaradio_debug)
					ast_log(LOG_ERROR, "XRUN read\n");
				snd_pcm_prepare(o->inhandle);
			} else if (r == -ESTRPIPE) {
				ast_log(LOG_ERROR, "-ESTRPIPE\n");
				snd_pcm_prepare(o->inhandle);
			} else if (r < 0) {
				ast_log(LOG_ERROR, "Read error: %s\n", snd_strerror(r));
			} else
				alsa_monitor_read((char *) buf, r * 2);
		}
#endif
		if (FD_ISSET(o->sndcmd[0], &rfds)) {
			read(o->sndcmd[0], &o->cursound, sizeof(o->cursound));
		}
		if (FD_ISSET(o->outdev, &wfds))
			send_sound(o);
	}
	/* Never reached */
	return NULL;
}


#endif

/*
 * reset and close the device if opened,
 * then open and initialize it in the desired mode,
 * trigger reads and writes so we can start using it.
 */
static int setformat(struct chan_alsaradio_pvt *o, int mode)
{
	alsa_card_uninit(o);
	serial_uninit(o);

	switch(mode){
		case O_RDWR:
			if (alsa_card_init(o, o->indevname, SND_PCM_STREAM_CAPTURE) == NULL)
			{
				ast_log(LOG_ERROR, "Problem opening ALSA IN device: %s\n", o->indevname);
				alsa_card_uninit(o);
				return -1;
			}
			if (alsa_card_init(o, o->outdevname, SND_PCM_STREAM_PLAYBACK) == NULL)
			{
				ast_log(LOG_ERROR, "Problem opening ALSA OUT device: %s\n", o->outdevname);
				alsa_card_uninit(o);
				return -1;
			}
			serial_init(o);
			break;
		case O_WRONLY:
			if (alsa_card_init(o, o->outdevname, SND_PCM_STREAM_PLAYBACK) == NULL)
			{
				ast_log(LOG_ERROR, "Problem opening ALSA IN device: %s\n", o->outdevname);
				alsa_card_uninit(o);
				return -1;
			}
			serial_init(o);
			break;
		case O_RDONLY:
			if (alsa_card_init(o, o->indevname, SND_PCM_STREAM_CAPTURE) == NULL)
			{
				ast_log(LOG_ERROR, "Problem opening ALSA IN device: %s\n", o->indevname);
				alsa_card_uninit(o);
				return -1;
			}
			serial_init(o);
			break;
		case O_CLOSE:
			alsa_card_uninit(o);
			serial_uninit(o);
			break;
		default:
			alsa_card_uninit(o);
			serial_uninit(o);
			ast_log(LOG_ERROR, "Mode not known: %s\n", o->indevname);
	}

	o->sounddev = o->indev;

	return (o->sounddev);
}

/*
 * some of the standard methods supported by channels.
 */
static int alsaradio_digit_begin(struct ast_channel *c, char digit)
{
	return 0;
}

static int alsaradio_digit_end(struct ast_channel *c, char digit, unsigned int duration)
{
	/* no better use for received digits than print them */
	ast_verbose(" << Console Received digit %c of duration %u ms >> \n", 
		digit, duration);
	return 0;
}

static int alsaradio_text(struct ast_channel *c, const char *text)
{
	struct chan_alsaradio_pvt *o = c->tech_pvt;
	char *cmd;

	cmd = alloca(strlen(text) + 10);

	/* print received messages */
	if(o->debuglevel)ast_verbose(" << Console Received alsaradio text %s >> \n", text);

	return 0;
}

/* Play ringtone 'x' on device 'o' */
static void ring(struct chan_alsaradio_pvt *o, int x)
{
#ifndef	NEW_ASTERISK
	write(o->sndcmd[1], &x, sizeof(x));
#endif
}

/*
 * handler for incoming calls. Either autoanswer, or start ringing
 */
static int alsaradio_call(struct ast_channel *c, char *dest, int timeout)
{
	struct chan_alsaradio_pvt *o = c->tech_pvt;

	ast_verbose("alsaradio -- call\n");

	o->stopser = 0;
	time(&o->lastsertime);
	ast_pthread_create_background(&o->serthread, NULL, serthread, o);
	ast_setstate(c, AST_STATE_UP);

	ast_mutex_lock(&alsalock);

	snd_pcm_prepare(o->inhandle);
        snd_pcm_start(o->inhandle);

	snd_pcm_prepare(o->outhandle);
        snd_pcm_start(o->outhandle);

        ast_mutex_unlock(&alsalock);

	return 0;
}

/*
 * remote side answered the phone
 */
static int alsaradio_answer(struct ast_channel *c)
{
#ifndef	NEW_ASTERISK
	struct chan_alsaradio_pvt *o = c->tech_pvt;
#endif

	ast_setstate(c, AST_STATE_UP);
#ifndef	NEW_ASTERISK
	o->cursound = -1;
	o->nosound = 0;
#endif
	return 0;
}

static int alsaradio_hangup(struct ast_channel *c)
{
	struct chan_alsaradio_pvt *o = c->tech_pvt;

	ast_log(LOG_NOTICE, "alsaradio_hangup()\n");
#ifndef	NEW_ASTERISK
	o->cursound = -1;
	o->nosound = 0;
#endif
	c->tech_pvt = NULL;
	o->owner = NULL;
	ast_module_unref(ast_module_info->self);
	if (o->hookstate) {
		if (o->autoanswer || o->autohangup) {
			/* Assume auto-hangup too */
			o->hookstate = 0;
			setformat(o, O_CLOSE);
		} else {
			/* Make congestion noise */
			ring(o, AST_CONTROL_CONGESTION);
		}
	}
	o->stopser = 1;
	o->stopwrite = 1;
	pthread_join(o->serthread,NULL);
	return 0;
}

/*
 * Write exactly one frame.
 */
static int alsa_write(struct chan_alsaradio_pvt *o, struct ast_frame *f)
{
	static char sizbuf[8000];
	static int sizpos = 0;
	int len = sizpos;
	int pos;
	int res = 0;
	/* size_t frames = 0; */
	snd_pcm_state_t state;

	ast_mutex_lock(&alsalock);

	/* We have to digest the frame in 160-byte portions */
	if (f->datalen > sizeof(sizbuf) - sizpos) {
		ast_log(LOG_WARNING, "Frame too large\n");
		res = -1;
	} else {
		memcpy(sizbuf + sizpos, f->data, f->datalen);
		len += f->datalen;
		pos = 0;

		state = snd_pcm_state(o->outhandle);
		if (state == SND_PCM_STATE_XRUN)
			snd_pcm_prepare(o->outhandle);
		while ((res = snd_pcm_writei(o->outhandle, sizbuf, len / 2)) == -EAGAIN) {
			usleep(1);
		}
		if (res == -EPIPE) {
			if (alsaradio_debug)
				ast_log(LOG_WARNING, "XRUN write\n");
			snd_pcm_prepare(o->outhandle);
			while ((res = snd_pcm_writei(o->outhandle, sizbuf, len / 2)) == -EAGAIN) {
				usleep(1);
			}
			if (res != len / 2) {
				ast_log(LOG_ERROR, "Write error: %s\n", snd_strerror(res));
				res = -1;
			} else if (res < 0) {
				ast_log(LOG_ERROR, "Write error %s\n", snd_strerror(res));
				res = -1;
			}
		} else {
			if (res == -ESTRPIPE)
				ast_log(LOG_ERROR, "You've got some big problems\n");
			else if (res < 0)
				ast_log(LOG_NOTICE, "Error %d on write\n", res);
		}
	}
	ast_mutex_unlock(&alsalock);

	return res;
}

/* used for data coming from the network */
static int alsaradio_write(struct ast_channel *c, struct ast_frame *f)
{
	struct chan_alsaradio_pvt *o = c->tech_pvt;
	struct ast_frame *f1;
	int i, n;

	short buftmp[8000];
	FILE *ftmp;
	int ret;
	struct timeval tv;
	struct timeval tv2;

#ifndef	NEW_ASTERISK
	/* Immediately return if no sound is enabled */
	if (o->nosound)
		return 0;
	/* Stop any currently playing sound */
	o->cursound = -1;
#endif
	/*
	 * we could receive a block which is not a multiple of our
	 * FRAME_SIZE, so buffer it locally and write to the device
	 * in FRAME_SIZE chunks.
	 * Keep the residue stored for future use.
	 */

	#if DEBUG_CAPTURES == 1	// to write input data to a file   datalen=320
	if (ftxcapraw && o->b.txcapraw)
	{
		short i, tbuff[f->datalen];
		int res;
		for(i=0;i<f->datalen;i+=2)
		{
			tbuff[i]= ((short*)(f->data))[i/2];
			tbuff[i+1]= o->txkeyed*0x1000;
		}
		if ((res = fwrite(tbuff,2,f->datalen,ftxcapraw)) < -1)
			ast_log(LOG_ERROR, "tx capture error\n");
	}
	#endif

#if 0
	tv = ast_tvnow();
	ast_log(LOG_ERROR, "time: %i\n", tv.tv_usec - o->tv.tv_usec);
	o->tv = tv;

	o->tv2 = ast_tvnow();
	alsa_write(o, f);
	tv2 = ast_tvnow();
	ast_log(LOG_ERROR, "time2: %i\n", tv2.tv_usec - o->tv2.tv_usec);
	o->tv2 = tv2;
#endif
	//ast_log(LOG_ERROR, "alsaradio_write\n");

	if (!o->txkeyed) return 0;

	f1 = ast_frdup(f);
	memset(&f1->frame_list,0,sizeof(f1->frame_list));
	ast_mutex_lock(&o->txqlock);
	AST_LIST_INSERT_TAIL(&o->txq,f1,frame_list);
	ast_mutex_unlock(&o->txqlock);

	n = 0;
	ast_mutex_lock(&o->txqlock);
	AST_LIST_TRAVERSE(&o->txq, f1,frame_list) n++;
	ast_mutex_unlock(&o->txqlock);

	/* depending on n we could do filtering */
	if (n < 1) return 0;

	for (i = 0; i < n; i++)
	{
		ast_mutex_lock(&o->txqlock);
		f1 = AST_LIST_REMOVE_HEAD(&o->txq,frame_list);
		ast_mutex_unlock(&o->txqlock);

		alsa_write(o, f1);

		ast_frfree(f1);
	}

	return 0;
}

/*
 * Read available data and return a frame (null frame if not enough data).
 */
static struct ast_frame *alsa_read(struct chan_alsaradio_pvt *o)
{
	static struct ast_frame f;
	static short __buf[FRAME_SIZE + AST_FRIENDLY_OFFSET / 2];
	short *buf;
	static int readpos = 0;
	static int left = FRAME_SIZE;
	snd_pcm_state_t state;
	int r = 0;
	int off = 0;

	ast_mutex_lock(&alsalock);
	f.frametype = AST_FRAME_NULL;
	f.subclass = 0;
	f.samples = 0;
	f.datalen = 0;
	f.data = NULL;
	f.offset = 0;
	f.src = alsaradio_tech.type;
	f.mallocd = 0;
	f.delivery.tv_sec = 0;
	f.delivery.tv_usec = 0;

	state = snd_pcm_state(o->inhandle);
	if ((state != SND_PCM_STATE_PREPARED) && (state != SND_PCM_STATE_RUNNING)) {
		snd_pcm_prepare(o->inhandle);
	}

	buf = __buf + AST_FRIENDLY_OFFSET / 2;

	r = snd_pcm_readi(o->inhandle, buf + readpos, left);
	if (r == -EPIPE) {
		if (alsaradio_debug)
			ast_log(LOG_ERROR, "XRUN read\n");
		snd_pcm_prepare(o->inhandle);
	} else if (r == -ESTRPIPE) {
		ast_log(LOG_ERROR, "-ESTRPIPE\n");
		snd_pcm_prepare(o->inhandle);
	} else if (r < 0) {
		ast_log(LOG_ERROR, "Read error: %s\n", snd_strerror(r));
	} else if (r >= 0) {
		off -= r;
	}
	/* Update positions */
	readpos += r;
	left -= r;

	if (readpos >= FRAME_SIZE) {
		/* A real frame */
		readpos = 0;
		left = FRAME_SIZE;
		f.frametype = AST_FRAME_VOICE;
		f.subclass = AST_FORMAT_SLINEAR;
		f.samples = FRAME_SIZE;
		f.datalen = FRAME_SIZE * 2;
		f.data = buf;
		f.offset = AST_FRIENDLY_OFFSET;
		f.src = alsaradio_tech.type;
		f.mallocd = 0;

	}
	ast_mutex_unlock(&alsalock);

	return &f;
}


static struct ast_frame *alsaradio_read(struct ast_channel *c)
{
	int cd,sd;
	struct chan_alsaradio_pvt *o = c->tech_pvt;
	struct ast_frame *f = &o->read_f,*f1;
	struct ast_frame wf = { AST_FRAME_CONTROL };
	time_t now;

	if (o->lastsertime)
	{
		time(&now);
		if ((now - o->lastsertime) > 3)
		{
			ast_log(LOG_ERROR,"SER process has died or something!!\n");
			return NULL;
		}
	}


	f = alsa_read(o);

	if (!o->txkeyed)
        {
                struct timeval tv, tv2;

                tv = ast_tvnow();
                //f = alsa_read(o);
                tv2 = ast_tvnow();
		//if (( tv2.tv_usec - o->tv.tv_usec) > 20000)
		//ast_log(LOG_ERROR, "time: %i, %i\n", tv2.tv_usec - o->tv.tv_usec, tv2.tv_usec - tv.tv_usec);
		o->tv = tv2;
	}
	/* if we don't have enough data just return the NULL frame*/
	if (f->frametype == AST_FRAME_NULL)
		return f;

	#if DEBUG_CAPTURES == 1
	if (o->b.rxcapraw && frxcapraw) fwrite(f->data,1,f->datalen,frxcapraw);
	#endif

	if (o->mute)
		return f;

	cd = 1; /* assume CD */
	if ((o->rxcdtype == SERIAL_DSR) && (!o->rxsersq)) cd = 0;
	else if ((o->rxcdtype == SERIAL_DSR_INVERT) && o->rxsersq) cd = 0;

	/* apply cd turn-on delay, if one specified */
	if (o->rxondelay && cd && (o->rxoncnt++ < o->rxondelay)) cd = 0;
	else if (!cd) o->rxoncnt = 0;

	sd = 1; /* assume SD */
	if ((o->rxsdtype == SERIAL_CTS) && (!o->rxserctcss)) sd = 0;
	else if ((o->rxsdtype == SERIAL_CTS_INVERT) && o->rxserctcss) sd = 0;

	o->rxkeyed = sd && cd && ((!o->lasttx) || o->duplex);

	if (o->lastrx && (!o->rxkeyed))
	{
		o->lastrx = 0;
		wf.subclass = AST_CONTROL_RADIO_UNKEY;
		ast_queue_frame(o->owner, &wf);
	}
	else if ((!o->lastrx) && (o->rxkeyed))
	{
		o->lastrx = 1;
		wf.subclass = AST_CONTROL_RADIO_KEY;
		ast_queue_frame(o->owner, &wf);
	}

        if (c->_state != AST_STATE_UP)  /* drop data if frame is not up */
                return f;

	if (o->usedtmf && o->dsp)
	{
	    f1 = ast_dsp_process(c,o->dsp,f);
	    if ((f1->frametype == AST_FRAME_DTMF_END) ||
	      (f1->frametype == AST_FRAME_DTMF_BEGIN))
	    {
		if ((f1->subclass == 'm') || (f1->subclass == 'u'))
		{
			f1->frametype = AST_FRAME_NULL;
			f1->subclass = 0;
			return(f1);
		}
		if (f1->frametype == AST_FRAME_DTMF_END)
			ast_log(LOG_NOTICE,"Got DTMF char %c\n",f1->subclass);
		return(f1);
	    }
	}
        if (o->boost != BOOST_SCALE) {  /* scale and clip values */
                int i, x;
                int16_t *p = (int16_t *) f->data;
                for (i = 0; i < f->samples; i++) {
                        x = (p[i] * o->boost) / BOOST_SCALE;
                        if (x > 32767)
                                x = 32767;
                        else if (x < -32768)
                                x = -32768;
                        p[i] = x;
                }
        }
        f->offset = AST_FRIENDLY_OFFSET;
        return f;
}

static int alsaradio_fixup(struct ast_channel *oldchan, struct ast_channel *newchan)
{
	struct chan_alsaradio_pvt *o = newchan->tech_pvt;
	ast_log(LOG_WARNING,"alsaradio_fixup()\n");
	o->owner = newchan;
	return 0;
}

static int alsaradio_indicate(struct ast_channel *c, int cond, const void *data, size_t datalen)
{
	struct chan_alsaradio_pvt *o = c->tech_pvt;
	int res = -1;

	switch (cond) {
		case AST_CONTROL_BUSY:
		case AST_CONTROL_CONGESTION:
		case AST_CONTROL_RINGING:
			res = cond;
			break;

		case -1:
#ifndef	NEW_ASTERISK
			o->cursound = -1;
			o->nosound = 0;		/* when cursound is -1 nosound must be 0 */
#endif
			return 0;

		case AST_CONTROL_VIDUPDATE:
			res = -1;
			break;
		case AST_CONTROL_HOLD:
			ast_verbose(" << Console Has Been Placed on Hold >> \n");
			ast_moh_start(c, data, o->mohinterpret);
			break;
		case AST_CONTROL_UNHOLD:
			ast_verbose(" << Console Has Been Retrieved from Hold >> \n");
			ast_moh_stop(c);
			break;
		case AST_CONTROL_PROCEEDING:
			ast_verbose(" << Call Proceeding... >> \n");
			ast_moh_stop(c);
			break;
		case AST_CONTROL_PROGRESS:
			ast_verbose(" << Call Progress... >> \n");
			ast_moh_stop(c);
			break;
		case AST_CONTROL_RADIO_KEY:
			o->txkeyed = 1;
			kickptt(o);
			if(o->debuglevel)ast_verbose("chan_alsaradio ACRK  dev=%s TX ON \n",o->name);
			break;
		case AST_CONTROL_RADIO_UNKEY:
			o->txkeyed = 0;
			kickptt(o);
			/* TODO - clenup the tx list */
			if(o->debuglevel)ast_verbose("chan_alsaradio ACRUK  dev=%s TX OFF >> \n",o->name);
			break;
		default:
			ast_log(LOG_WARNING, "Don't know how to display condition %d on %s\n", cond, c->name);
			return -1;
	}

	if (res > -1)
		ring(o, res);

	return 0;
}

static int alsaradio_setoption(struct ast_channel *chan, int option, void *data, int datalen)
{
	char *cp;
	struct chan_alsaradio_pvt *o = chan->tech_pvt;

	/* all supported options require data */
	if (!data || (datalen < 1)) {
		errno = EINVAL;
		return -1;
	}

	switch (option) {
	case AST_OPTION_TONE_VERIFY:
		cp = (char *) data;
		switch (*cp) {
		case 1:
			ast_log(LOG_DEBUG, "Set option TONE VERIFY, mode: OFF(0) on %s\n",chan->name);
			o->usedtmf = 1;
			break;
		case 2:
			ast_log(LOG_DEBUG, "Set option TONE VERIFY, mode: MUTECONF/MAX(2) on %s\n",chan->name);
			o->usedtmf = 1;
			break;
		case 3:
			ast_log(LOG_DEBUG, "Set option TONE VERIFY, mode: DISABLE DETECT(3) on %s\n",chan->name);
			o->usedtmf = 0;
			break;
		default:
			ast_log(LOG_DEBUG, "Set option TONE VERIFY, mode: OFF(0) on %s\n",chan->name);
			o->usedtmf = 1;
			break;
		}
		break;
	}
	errno = 0;
	return 0;
}

/*
 * allocate a new channel.
 */
static struct ast_channel *alsaradio_new(struct chan_alsaradio_pvt *o, char *ext, char *ctx, int state)
{
	struct ast_channel *c;

	c = ast_channel_alloc(1, state, o->cid_num, o->cid_name, "", ext, ctx, 0, "alsaradio/%s", o->name);
	if (c == NULL)
		return NULL;
	c->tech = &alsaradio_tech;
	if (o->sounddev < 0) 
		setformat(o, O_RDWR);
	c->fds[0] = o->sounddev;	/* -1 if device closed, override later */
	c->nativeformats = AST_FORMAT_SLINEAR;
	c->readformat = AST_FORMAT_SLINEAR;
	c->writeformat = AST_FORMAT_SLINEAR;
	c->tech_pvt = o;

	if (!ast_strlen_zero(o->language))
		ast_string_field_set(c, language, o->language);
	/* Don't use ast_set_callerid() here because it will
	 * generate a needless NewCallerID event */
	c->cid.cid_num = ast_strdup(o->cid_num);
	c->cid.cid_ani = ast_strdup(o->cid_num);
	c->cid.cid_name = ast_strdup(o->cid_name);
	if (!ast_strlen_zero(ext))
		c->cid.cid_dnid = ast_strdup(ext);

	o->owner = c;
	ast_module_ref(ast_module_info->self);
	ast_jb_configure(c, &global_jbconf);
	if (state != AST_STATE_DOWN) {
		if (ast_pbx_start(c)) {
			ast_log(LOG_WARNING, "Unable to start PBX on %s\n", c->name);
			ast_hangup(c);
			o->owner = c = NULL;
			/* XXX what about the channel itself ? */
			/* XXX what about usecnt ? */
		}
	}

	return c;
}
/*
*/
static struct ast_channel *alsaradio_request(const char *type, int format, void *data, int *cause)
{
	struct ast_channel *c;
	struct chan_alsaradio_pvt *o = find_desc(data);

	if (1)
	{
		ast_log(LOG_WARNING, "alsaradio_request type <%s> data 0x%p <%s>\n", type, data, (char *) data);
	}
	if (o == NULL) {
		ast_log(LOG_NOTICE, "Device %s not found\n", (char *) data);
		/* XXX we could default to 'dsp' perhaps ? */
		return NULL;
	}
	if ((format & AST_FORMAT_SLINEAR) == 0) {
		ast_log(LOG_NOTICE, "Format 0x%x unsupported\n", format);
		return NULL;
	}
	if (o->owner) {
		ast_log(LOG_NOTICE, "Already have a call (chan %p) on the aradio channel\n", o->owner);
		*cause = AST_CAUSE_BUSY;
		return NULL;
	}
	c = alsaradio_new(o, NULL, NULL, AST_STATE_DOWN);
	if (c == NULL) {
		ast_log(LOG_WARNING, "Unable to create new aradio channel\n");
		return NULL;
	}
		
	return c;
}
/*
*/
static int console_key(int fd, int argc, char *argv[])
{
	struct chan_alsaradio_pvt *o = find_desc(alsaradio_active);

	if (argc != 2)
		return RESULT_SHOWUSAGE; 
	o->txtestkey = 1;
	kickptt(o);
	return RESULT_SUCCESS;
}
/*
*/
static int console_unkey(int fd, int argc, char *argv[])
{
	struct chan_alsaradio_pvt *o = find_desc(alsaradio_active);

	if (argc != 2)
		return RESULT_SHOWUSAGE;
	o->txtestkey = 0;
	kickptt(o);
	return RESULT_SUCCESS;
}


static int console_rkey(int fd, int argc, char *argv[])
{
	sim_cor = 1;

	return 0;
}

static int console_runkey(int fd, int argc, char *argv[])
{
	sim_cor = 0;

	return 0;
}

static int radio_tune(int fd, int argc, char *argv[])
{
	struct chan_alsaradio_pvt *o = find_desc(alsaradio_active);
	int i=0;

	if ((argc < 2) || (argc > 4))
		return RESULT_SHOWUSAGE; 

	if (argc == 2) /* just show stuff */
	{
		ast_cli(fd,"Active radio interface is [%s]\n",alsaradio_active);
		ast_cli(fd,"Device String is %s\n",o->devstr);
		ast_cli(fd,"Rx Level currently set to %d\n",o->rxmixerset);
		ast_cli(fd,"Tx Output A Level currently set to %d\n",o->txmixaset);
		ast_cli(fd,"Tx Output B Level currently set to %d\n",o->txmixbset);
		return RESULT_SHOWUSAGE;
	}

	else if (!strcasecmp(argv[2],"rx")) {
		i = 0;

		if (argc == 3)
		{
			ast_cli(fd,"Current setting on Rx Channel is %d\n",o->rxmixerset);
		}
		else
		{
			i = atoi(argv[3]);
			if ((i < 0) || (i > 999)) return RESULT_SHOWUSAGE;
		 	o->rxmixerset = i;
			ast_cli(fd,"Changed setting on RX Channel to %d\n",o->rxmixerset);
			mixer_write(o);
		}
	}
	else if (!strcasecmp(argv[2],"txa")) {
		i = 0;

		if (argc == 3)
		{
			ast_cli(fd,"Current setting on Tx Channel A is %d\n",o->txmixaset);
		}
		else
		{
			i = atoi(argv[3]);
			if ((i < 0) || (i > 999)) return RESULT_SHOWUSAGE;
		 	o->txmixaset = i;
			ast_cli(fd,"Changed setting on TX Channel A to %d\n",o->txmixaset);
			mixer_write(o);
		}
	}
	else if (!strcasecmp(argv[2],"txb")) {
		i = 0;

		if (argc == 3)
		{
			ast_cli(fd,"Current setting on Tx Channel A is %d\n",o->txmixbset);
		}
		else
		{
			i = atoi(argv[3]);
			if ((i < 0) || (i > 999)) return RESULT_SHOWUSAGE;
		 	o->txmixbset = i;
			ast_cli(fd,"Changed setting on TX Channel A to %d\n",o->txmixbset);
			mixer_write(o);
		}
	}
	else if (!strcasecmp(argv[2],"nocap")) 	
	{
		ast_cli(fd,"File capture (raw)   was rx=%d tx=%d and now off.\n",o->b.rxcapraw,o->b.txcapraw);
		o->b.rxcapraw=o->b.txcapraw=0;
		if (frxcapraw) { fclose(frxcapraw); frxcapraw = NULL; }
		if (ftxcapraw) { fclose(ftxcapraw); ftxcapraw = NULL; }
	}
	else if (!strcasecmp(argv[2],"rxcap")) 
	{
		if (!frxcapraw) frxcapraw = fopen(RX_CAP_RAW_FILE,"w");
		ast_cli(fd,"cap rx raw on.\n");
		o->b.rxcapraw=1;
	}
	else if (!strcasecmp(argv[2],"txcap")) 
	{
	if (!ftxcapraw) ftxcapraw = fopen(TX_CAP_RAW_FILE,"w");
		ast_cli(fd,"cap tx raw on.\n");
		o->b.txcapraw=1;
	}
	else
        {
                return RESULT_SHOWUSAGE;
        }
	return RESULT_SUCCESS;
}

/*
	CLI debugging on and off
*/
static int radio_set_debug(int fd, int argc, char *argv[])
{
	struct chan_alsaradio_pvt *o = find_desc(alsaradio_active);

	o->debuglevel=1;
	ast_cli(fd,"aradio debug on.\n");
	return RESULT_SUCCESS;
}

static int radio_set_debug_off(int fd, int argc, char *argv[])
{
	struct chan_alsaradio_pvt *o = find_desc(alsaradio_active);

	o->debuglevel=0;
	ast_cli(fd,"aradio debug off.\n");
	return RESULT_SUCCESS;
}

static int radio_active(int fd, int argc, char *argv[])
{
        if (argc == 2)
                ast_cli(fd, "active (command) active ALSA device is [%s]\n", alsaradio_active);
        else if (argc != 3)
                return RESULT_SHOWUSAGE;
        else {
                struct chan_alsaradio_pvt *o;
                if (strcmp(argv[2], "show") == 0) {
                        for (o = alsaradio_default.next; o; o = o->next)
                                ast_cli(fd, "device [%s] exists as device=%s\n", 
					o->name,o->devstr);
                        return RESULT_SUCCESS;
                }
                o = find_desc(argv[2]);
                if (o == NULL)
                        ast_cli(fd, "No device [%s] exists\n", argv[2]);
                else
		{
                    alsaradio_active = o->name;
		}
        }
        return RESULT_SUCCESS;
}

static char key_usage[] =
	"Usage: aradio key\n"
	"       Simulates radio PTT keyed.\n";

static char unkey_usage[] =
	"Usage: aradio unkey\n"
	"       Simulates radio PTT unkeyed.\n";

static char rkey_usage[] =
	"Usage: aradio rkey\n"
	"       Simulates radio COR active.\n";

static char runkey_usage[] =
	"Usage: aradio runkey\n"
	"       Simulates radio COR inactive.\n";

static char active_usage[] =
        "Usage: aradio active [device-name]\n"
        "       If used without a parameter, displays which device is the current\n"
        "one being commanded.  If a device is specified, the commanded radio device is changed\n"
        "to the device specified.\n";
/*
radio tune 6 3000		measured tx value
*/
static char radio_tune_usage[] =
	"Usage: aradio tune <function>\n"
	"       rx [newsetting]\n"
	"       txa [newsetting]\n"
	"       txb [newsetting]\n"
	"       save (settings to tuning file)\n"
	"       load (tuning settings from EEPROM)\n"
	"\n       All [newsetting]'s are values 0-999\n\n";
					  
#ifndef	NEW_ASTERISK

static struct ast_cli_entry cli_alsaradio[] = {
	{ { "aradio", "key", NULL },
	console_key, "Radio PTT Key",
	key_usage, NULL, NULL},

	{ { "aradio", "unkey", NULL },
	console_unkey, "Radio PTT Unkey",
	unkey_usage, NULL, NULL },

	{ { "aradio", "rkey", NULL },
	console_rkey, "Radio COR active",
	rkey_usage, NULL, NULL},

	{ { "aradio", "runkey", NULL },
	console_runkey, "Radio COR inactive",
	runkey_usage, NULL, NULL },

	{ { "aradio", "tune", NULL },
	radio_tune, "Radio Tune",
	radio_tune_usage, NULL, NULL },

	{ { "aradio", "set", "debug", NULL },
	radio_set_debug, "Radio Debug",
	radio_tune_usage, NULL, NULL },

	{ { "aradio", "set", "debug", "off", NULL },
	radio_set_debug_off, "Radio Debug",
	radio_tune_usage, NULL, NULL },

	{ { "aradio", "active", NULL },
	radio_active, "Change commanded device",
	active_usage, NULL, NULL },

};
#endif

static void store_rxcdtype(struct chan_alsaradio_pvt *o, char *s)
{
	if (!strcasecmp(s,"no")){
		o->rxcdtype = CD_IGNORE;
	}
	else if (!strcasecmp(s,"serialdsr")){
		o->rxcdtype = SERIAL_DSR;
	}
	else if (!strcasecmp(s,"serialdsrinvert")){
		o->rxcdtype = SERIAL_DSR_INVERT;
	}	
	else {
		ast_log(LOG_WARNING,"Unrecognized rxcdtype parameter: %s\n",s);
	}

	ast_log(LOG_WARNING, "set rxcdtype = %s\n", s);
}
/*
*/
static void store_rxsdtype(struct chan_alsaradio_pvt *o, char *s)
{
	if (!strcasecmp(s,"no")){
		o->rxsdtype = SD_IGNORE;
	}
	else if (!strcasecmp(s,"serialcts")){
		o->rxsdtype = SERIAL_CTS;
	}
	else if (!strcasecmp(s,"serialctsinvert")){
		o->rxsdtype = SERIAL_CTS_INVERT;
	}	
	else {
		ast_log(LOG_WARNING,"Unrecognized rxsdtype parameter: %s\n",s);
	}

	ast_log(LOG_WARNING, "set rxsdtype = %s\n", s);
}

static void tune_write(struct chan_alsaradio_pvt *o)
{
	FILE *fp;
	char fname[200];

 	snprintf(fname,sizeof(fname) - 1,"/etc/asterisk/alsaradio_tune_%s.conf",o->name);
	fp = fopen(fname,"w");

	fprintf(fp,"[%s]\n",o->name);

	fprintf(fp,"; name=%s\n",o->name);
	fprintf(fp,"; devicenum=%i\n",o->devicenum);
	fprintf(fp,"devstr=%s\n",o->devstr);
	fprintf(fp,"rxmixerset=%i\n",o->rxmixerset);
	fprintf(fp,"txmixaset=%i\n",o->txmixaset);
	fprintf(fp,"txmixbset=%i\n",o->txmixbset);
	fclose(fp);

}
//
static void mixer_write(struct chan_alsaradio_pvt *o)
{
	//setamixer(o->devicenum,MIXER_PARAM_MIC_PLAYBACK_SW,0,0);
	//setamixer(o->devicenum,MIXER_PARAM_MIC_PLAYBACK_VOL,0,0);
	//setamixer(o->devicenum,MIXER_PARAM_SPKR_PLAYBACK_SW,1,0);
	//setamixer(o->devicenum,MIXER_PARAM_SPKR_PLAYBACK_VOL,
	//	make_spkr_playback_value(o,o->txmixaset),
	//	make_spkr_playback_value(o,o->txmixbset));
	//setamixer(o->devicenum,MIXER_PARAM_MIC_CAPTURE_VOL,
	//	o->rxmixerset * o->micmax / 1000,0);
	//setamixer(o->devicenum,MIXER_PARAM_MIC_BOOST,o->rxboostset,0);
	//setamixer(o->devicenum,MIXER_PARAM_MIC_CAPTURE_SW,1,0);
}
/*
	adjust dsp multiplier to add resolution to tx level adjustment
*/


/*
 * grab fields from the config file, init the descriptor and open the device.
 */
static struct chan_alsaradio_pvt *store_config(struct ast_config *cfg, char *ctg)
{
	struct ast_variable *v;
	struct chan_alsaradio_pvt *o;
#ifdef	NEW_ASTERISK
	struct ast_flags zeroflag = {0};
#endif
	if (ctg == NULL) {
		o = &alsaradio_default;
		ctg = "general";
	} else {
		/* "general" is also the default thing */
		if (strcmp(ctg, "general") == 0) {
			o = &alsaradio_default;
		} else {
		    	ast_log(LOG_NOTICE,"ast_calloc for chan_alsaradio_pvt of %s\n",ctg);
			if (!(o = ast_calloc(1, sizeof(*o))))
				return NULL;
			*o = alsaradio_default;
			o->name = ast_strdup(ctg);
			if (!alsaradio_active) 
				alsaradio_active = o->name;
		}
	}
	ast_mutex_init(&o->txqlock);
	strcpy(o->mohinterpret, "default");
	/* fill other fields from configuration */
	for (v = ast_variable_browse(cfg, ctg); v; v = v->next) {
		M_START((char *)v->name, (char *)v->value);

		/* handle jb conf */
		if (!ast_jb_read_conf(&global_jbconf, v->name, v->value))
			continue;

			M_UINT("frags", o->frags)
			M_UINT("queuesize",o->queuesize)
			M_UINT("debug", alsaradio_debug)
			M_BOOL("rxcpusaver",o->rxcpusaver)
			M_BOOL("txcpusaver",o->txcpusaver)
			M_BOOL("invertptt",o->invertptt)
			M_F("carrierfrom",store_rxcdtype(o,(char *)v->value))
			M_F("ctcssfrom",store_rxsdtype(o,(char *)v->value))
 			M_BOOL("rxboost",o->rxboostset)
			M_UINT("duplex",o->radioduplex)

			/* ALSA stuff */
			M_BOOL("silencesuppression", o->silencesuppression)
			M_UINT("silencethreshold", o->silencethreshold)
			M_STR("input_device", o->indevname)
			M_STR("output_device", o->outdevname)
			M_STR("serial_device", o->serdevname)
			M_UINT("serial_disable", o->serdisable)
			M_UINT("rxondelay",o->rxondelay);

			M_END(;
			);
	}

	o->debuglevel=1;

	if (o == &alsaradio_default)		/* we are done with the default */
		return NULL;

	o->lastopen = ast_tvnow();	/* don't leave it 0 or tvdiff may wrap */
	o->dsp = ast_dsp_new();
	if (o->dsp)
	{
#ifdef  NEW_ASTERISK
          ast_dsp_set_features(o->dsp,DSP_FEATURE_DIGIT_DETECT);
          ast_dsp_set_digitmode(o->dsp,DSP_DIGITMODE_DTMF | DSP_DIGITMODE_MUTECONF | DSP_DIGITMODE_RELAXDTMF);
#else
          ast_dsp_set_features(o->dsp,DSP_FEATURE_DTMF_DETECT);
          ast_dsp_digitmode(o->dsp,DSP_DIGITMODE_DTMF | DSP_DIGITMODE_MUTECONF | DSP_DIGITMODE_RELAXDTMF);
#endif
	}

#ifndef	NEW_ASTERISK
	if (pipe(o->sndcmd) != 0) {
		ast_log(LOG_ERROR, "Unable to create pipe\n");
		goto error;
	}

	ast_pthread_create_background(&o->sthread, NULL, sound_thread, o);
#endif

	/* link into list of devices */
	if (o != &alsaradio_default) {
		o->next = alsaradio_default.next;
		alsaradio_default.next = o;
	}
	return o;
  
  error:
	if (o != &alsaradio_default)
		free(o);
	return NULL;
}

#ifdef	NEW_ASTERISK

static char *res2cli(int r)
{
	switch (r)
	{
	    case RESULT_SUCCESS:
		return(CLI_SUCCESS);
	    case RESULT_SHOWUSAGE:
		return(CLI_SHOWUSAGE);
	    default:
		return(CLI_FAILURE);
	}
}

static char *handle_console_key(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio key";
                e->usage = key_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(console_key(a->fd,a->argc,a->argv));
}

static char *handle_console_unkey(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio unkey";
                e->usage = unkey_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(console_unkey(a->fd,a->argc,a->argv));
}

static char *handle_console_rkey(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio rkey";
                e->usage = key_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(console_rkey(a->fd,a->argc,a->argv));
}

static char *handle_console_unkey(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio runkey";
                e->usage = unkey_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(console_unkey(a->fd,a->argc,a->argv));
}


static char *handle_aradio_tune(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio tune";
                e->usage = radio_tune_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(radio_tune(a->fd,a->argc,a->argv));
}

static char *handle_aradio_debug(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio debug";
                e->usage = radio_tune_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(radio_set_debug(a->fd,a->argc,a->argv));
}

static char *handle_aradio_debug_off(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio debug off";
                e->usage = radio_tune_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(radio_set_debug_off(a->fd,a->argc,a->argv));
}

static char *handle_aradio_active(struct ast_cli_entry *e,
	int cmd, struct ast_cli_args *a)
{
        switch (cmd) {
        case CLI_INIT:
                e->command = "aradio active";
                e->usage = active_usage;
                return NULL;
        case CLI_GENERATE:
                return NULL;
	}
	return res2cli(radio_active(a->fd,a->argc,a->argv));
}

static struct ast_cli_entry cli_alsaradio[] = {
	AST_CLI_DEFINE(handle_console_key,"Radio PTT key"),
	AST_CLI_DEFINE(handle_console_unkey,"Radio PTT  unkey"),
	AST_CLI_DEFINE(handle_console_rkey,"Radio COR active"),
	AST_CLI_DEFINE(handle_console_runkey,"Radio COR inactive"),
	AST_CLI_DEFINE(handle_aradio_tune,"aradio Tune"),
	AST_CLI_DEFINE(handle_aradio_debug,"aradio Debug On"),
	AST_CLI_DEFINE(handle_aradio_debug_off,"aradio Debug Off"),
	AST_CLI_DEFINE(handle_aradio_active,"Change commanded device")
};

#endif

static snd_pcm_t *alsa_card_init(struct chan_alsaradio_pvt *o, char *dev, snd_pcm_stream_t stream)
{
	int err;
	int direction;
	snd_pcm_t *handle = NULL;
	snd_pcm_hw_params_t *hwparams = NULL;
	snd_pcm_sw_params_t *swparams = NULL;
	struct pollfd pfd;
	snd_pcm_uframes_t period_size = FRAME_SIZE*2;
	snd_pcm_uframes_t buffer_size = 0;
	unsigned int rate = DESIRED_RATE;
	snd_pcm_uframes_t start_threshold, stop_threshold;

	err = snd_pcm_open(&handle, dev, stream, SND_PCM_NONBLOCK);
	if (err < 0) {
		ast_log(LOG_ERROR, "snd_pcm_open failed: %s\n", snd_strerror(err));
		return NULL;
	} else {
		ast_log(LOG_NOTICE, "Opening device %s in %s mode\n", dev, (stream == SND_PCM_STREAM_CAPTURE) ? "read" : "write");
	}

	hwparams = alloca(snd_pcm_hw_params_sizeof());
	memset(hwparams, 0, snd_pcm_hw_params_sizeof());
	snd_pcm_hw_params_any(handle, hwparams);

	err = snd_pcm_hw_params_set_access(handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (err < 0)
		ast_log(LOG_ERROR, "set_access failed: %s\n", snd_strerror(err));

	err = snd_pcm_hw_params_set_format(handle, hwparams, format);
	if (err < 0)
		ast_log(LOG_ERROR, "set_format failed: %s\n", snd_strerror(err));

	err = snd_pcm_hw_params_set_channels(handle, hwparams, 1);
	if (err < 0)
		ast_log(LOG_ERROR, "set_channels failed: %s\n", snd_strerror(err));

	direction = 0;
	err = snd_pcm_hw_params_set_rate_near(handle, hwparams, &rate, &direction);
	if (rate != DESIRED_RATE)
		ast_log(LOG_WARNING, "Rate not correct, requested %d, got %d\n", DESIRED_RATE, rate);

	direction = 0;
	err = snd_pcm_hw_params_set_period_size_near(handle, hwparams, &period_size, &direction);
	if (err < 0)
		ast_log(LOG_ERROR, "period_size(%ld frames) is bad: %s\n", period_size, snd_strerror(err));
	else {
		ast_log(LOG_NOTICE, "Period size is: %d\n", (int)period_size);
	}

	buffer_size = 4096 * 2;		/* period_size * 16; */
	err = snd_pcm_hw_params_set_buffer_size_near(handle, hwparams, &buffer_size);
	if (err < 0)
		ast_log(LOG_WARNING, "Problem setting buffer size of %ld: %s\n", buffer_size, snd_strerror(err));
	else {
		ast_log(LOG_NOTICE, "Buffer size is set to %d frames\n", (int)buffer_size);
	}

	err = snd_pcm_hw_params(handle, hwparams);
	if (err < 0)
		ast_log(LOG_ERROR, "Couldn't set the new hw params: %s\n", snd_strerror(err));

	swparams = alloca(snd_pcm_sw_params_sizeof());
	memset(swparams, 0, snd_pcm_sw_params_sizeof());
	snd_pcm_sw_params_current(handle, swparams);

	if (stream == SND_PCM_STREAM_PLAYBACK)
		start_threshold = period_size;
	else
		start_threshold = 1;

	err = snd_pcm_sw_params_set_start_threshold(handle, swparams, start_threshold);
	if (err < 0)
		ast_log(LOG_ERROR, "start threshold: %s\n", snd_strerror(err));

	if (stream == SND_PCM_STREAM_PLAYBACK)
		stop_threshold = buffer_size;
	else
		stop_threshold = buffer_size;

	err = snd_pcm_sw_params_set_stop_threshold(handle, swparams, stop_threshold);
	if (err < 0)
		ast_log(LOG_ERROR, "stop threshold: %s\n", snd_strerror(err));

	err = snd_pcm_sw_params(handle, swparams);
	if (err < 0)
		ast_log(LOG_ERROR, "sw_params: %s\n", snd_strerror(err));

	err = snd_pcm_poll_descriptors_count(handle);
	if (err <= 0)
		ast_log(LOG_ERROR, "Unable to get a poll descriptors count, error is %s\n", snd_strerror(err));
	if (err != 1) {
		ast_log(LOG_NOTICE, "Can't handle more than one device\n");
	}

	snd_pcm_poll_descriptors(handle, &pfd, err);
	ast_log(LOG_NOTICE, "Acquired fd %d from the poll descriptor\n", pfd.fd);

	if (stream == SND_PCM_STREAM_CAPTURE)
	{
		o->indev = pfd.fd;
		o->inhandle = handle;
	}
	else
	{
		o->outdev = pfd.fd;
		o->outhandle = handle;
	}

	return handle;
}


static void alsa_card_uninit(struct chan_alsaradio_pvt *o)
{
	if (o->inhandle)
                snd_pcm_close(o->inhandle);
        if (o->outhandle)
                snd_pcm_close(o->outhandle);

	o->sounddev = -1;
	o->indev = -1;
	o->outdev = -1;
}

static int serial_init(struct chan_alsaradio_pvt *o)
{
	int status;
	int ret;

	if (o->serdisable)
		return (0);

	if ((o->serdev = open(o->serdevname, O_RDWR)) < 0)
	{
		ast_log(LOG_ERROR, "Unable to open serial device %s: %s\n", o->serdevname, strerror(errno));
		return(-1);
	}

	if ((ret = ioctl(o->serdev, TIOCMGET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to get modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	/* set DTR active and RTS innactive */
        //status |= TIOCM_RTS;
        //status &= ~TIOCM_DTR;
	status |= TIOCM_DTR;
        status &= ~TIOCM_RTS;


	if ((ret = ioctl(o->serdev, TIOCMSET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to set modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	ast_log(LOG_NOTICE, "Serial device %s initialized\n", o->serdevname);

	return 0;
}

static void serial_uninit(struct chan_alsaradio_pvt *o)
{
	if (o->serdisable)
		return;

	if (o->serdev > 0)
	{
		close(o->serdev);
		o->serdev = -1;
	}

}

static int serial_getcor(struct chan_alsaradio_pvt *o)
{
	int ret, status;

	if (o->serdisable)
		return (0);

	if ((ret = ioctl(o->serdev, TIOCMGET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to get modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	//ast_log(LOG_NOTICE, "modem lines: %x\n", status);

	/* due to HW interface, COR signal is inverted */
	return (status & TIOCM_DSR) ? 0 : 1; 
}

static int serial_getctcss(struct chan_alsaradio_pvt *o)
{
	int ret, status;

	if (o->serdisable)
		return (0);

	if ((ret = ioctl(o->serdev, TIOCMGET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to get modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	return (status & TIOCM_CTS) ? 1 : 0; 
}

static int serial_pttkey(struct chan_alsaradio_pvt *o)
{
	int ret, status;

	if (o->serdisable)
		return (0);

	if ((ret = ioctl(o->serdev, TIOCMGET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to get modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	//status |= TIOCM_DTR;
	status |= TIOCM_RTS;

	if ((ret = ioctl(o->serdev, TIOCMSET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to set modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	return 0;
}

static int serial_pttunkey(struct chan_alsaradio_pvt *o)
{
	int ret, status;

	if (o->serdisable)
		return (0);

	if ((ret = ioctl(o->serdev, TIOCMGET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to get modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	//status &= ~TIOCM_DTR;
	status &= ~TIOCM_RTS;

	if ((ret = ioctl(o->serdev, TIOCMSET, &status)) < 0)
        {
                ast_log(LOG_ERROR, "Unable to set modem lines for %s: %s\n", o->serdevname, strerror(errno));
                return(-1);
        }

	return 0;
}


/*
*/
static int load_module(void)
{
	struct ast_config *cfg = NULL;
	char *ctg = NULL;
#ifdef	NEW_ASTERISK
	struct ast_flags zeroflag = {0};
#endif

	alsaradio_active = NULL;

	/* Copy the default jb config over global_jbconf */
	memcpy(&global_jbconf, &default_jbconf, sizeof(struct ast_jb_conf));

	/* load config file */
#ifdef	NEW_ASTERISK
	if (!(cfg = ast_config_load(config,zeroflag))) {
#else
	if (!(cfg = ast_config_load(config))) {
#endif
		ast_log(LOG_NOTICE, "Unable to load config %s\n", config);
		return AST_MODULE_LOAD_DECLINE;
	}

	do {
		store_config(cfg, ctg);
	} while ( (ctg = ast_category_browse(cfg, ctg)) != NULL);

	ast_config_destroy(cfg);

	if (ast_channel_register(&alsaradio_tech)) {
		ast_log(LOG_ERROR, "Unable to register channel type 'alsaradio'\n");
		return AST_MODULE_LOAD_FAILURE;
	}

	ast_cli_register_multiple(cli_alsaradio, sizeof(cli_alsaradio) / sizeof(struct ast_cli_entry));
	
	return AST_MODULE_LOAD_SUCCESS;
}

/*
*/
static int unload_module(void)
{
	struct chan_alsaradio_pvt *o;

	ast_log(LOG_WARNING, "unload_module() called\n");

	ast_channel_unregister(&alsaradio_tech);
	ast_cli_unregister_multiple(cli_alsaradio, sizeof(cli_alsaradio) / sizeof(struct ast_cli_entry));

	for (o = alsaradio_default.next; o; o = o->next) {

		#if DEBUG_CAPTURES == 1
		if (frxcapraw) { fclose(frxcapraw); frxcapraw = NULL; }
		if (ftxcapraw) { fclose(ftxcapraw); ftxcapraw = NULL; }
		#endif

		close(o->sounddev);
#ifndef	NEW_ASTERISK
		if (o->sndcmd[0] > 0) {
			close(o->sndcmd[0]);
			close(o->sndcmd[1]);
		}
#endif
		if (o->dsp) ast_dsp_free(o->dsp);
		if (o->owner)
			ast_softhangup(o->owner, AST_SOFTHANGUP_APPUNLOAD);
		if (o->owner)			/* XXX how ??? */
			return -1;

		serial_uninit(o);
	}
	return 0;
}


AST_MODULE_INFO_STANDARD(ASTERISK_GPL_KEY, "simple usb Radio Interface Channel Driver");

/*	end of file */


