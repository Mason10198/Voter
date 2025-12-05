/* 
* VOTER Client System Firmware for VOTER board
*
* Copyright (C) 2011-2015
* Jim Dixon, WB6NIL <jim@lambdatel.com>
* Copyright (C) 2016-2026
* Chuck Henderson, WB9UUS <wb9uus@liandee.com>
* Lee Woldanski, VE7FET <ve7fet@tparc.org>
* David Maciorowski, WA1JHK
* Mason Nelson, N5LSN <mason@n5lsn.com>
*
* This file is part of the VOTER System Project 
*
*   VOTER System is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 2 of the License, or
*   (at your option) any later version.

*   Voter System is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this project.  If not, see <http://www.gnu.org/licenses/>.
*
*   NOTE: Now works with latest (v3.30) MPLAB C30 compiler
*
*   ****mktime() is broken in MPLAB C30 for dates past 12/31/2020 23:59:59***
*   This version now uses an alternate routine in substitution for mktime(). 
*   All previous versions that use mktime() WILL be broken, not able to tell 
*   the current time. See https://www.microchip.com/forums/m653169.aspx
*/

/***********************************************************
For IMA ADPCM Codec:

Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
Netherlands.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its 
documentation for any purpose and without fee is hereby granted, 
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in 
supporting documentation, and that the names of Stichting Mathematisch
Centrum or CWI not be used in advertising or publicity pertaining to
distribution of the software without specific, written prior permission.

STICHTING MATHEMATISCH CENTRUM DISCLAIMS ALL WARRANTIES WITH REGARD TO
THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS, IN NO EVENT SHALL STICHTING MATHEMATISCH CENTRUM BE LIABLE
FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*******************************************************************
Note: It would have been nicer to (1) use G.726 rather than this
ancient version of ADPCM, but G.726 was a bit too computationally
complex for this hardware platform, and (2) *not* to have to transcode
the tx audio into mulaw before outputting it, but there is no room in
RAM for signed linear audio of the necessary buffer size; sigh!

******************************************************************/

#define THIS_IS_STACK_APPLICATION

#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <dsp.h>

// Include all headers for any enabled TCPIP Stack functions
#include "TCPIP Stack/TCPIP.h"

/* NOTE: By default, audio between the host and the client will be 
   encoded in ulaw, UNLESS specifically set by the adpcm option 
   in voter.conf on the host.
*/

/* VOTER (through-hole) runs on:	dsPIC33FJ128GP802
   RTCM (aka SMT_BOARD) runs on:	dsPIC33FJ128GP804
*/

/* Fortunately now, the problem that we had with weak signals producing RSSI readings all over the place
   (and was dealt with by using a longer-term, but less responsive average) is now fixed, thanks to Chuck,
   WB9UUS for pointing out the 16 bit value overflow that was being caused! It works much more better-er now,
   and produces rock-solid, stable results even on barely or non-readable signals. Old code has since been 
   removed, "Chuck Squelch" is now the default.
*/

/* Update the version number for the firmware here */
#define FIRMWARE_VERSION "4.00"

// Optional features - comment out to remove from build
#define DSPBEW

const ROM char VERSION[] = FIRMWARE_VERSION " (Compiled: " __DATE__ ")";

#define M_PI       3.14159265358979323846

#if defined(SMT_BOARD)

	#define	CTCSSIN	_RA7	// Pin 13
	#define	JP8	_RA8	// Pin 32 - Init EEPROM
	#define	JP9	_RA9	// Pin 35 - Calibrate Squelch
	#define	JP10 	_RA10	// Pin 12 - Calibrate Diode

	#define	JP11 	_RB9	// Pin 1 - Set LED3/4 for RX Level Mode
	
	#define	INITIALIZE 	JP8 	// Short JP8 on powerup to initialize EEPROM
	#define	INITIALIZE_WVF 	JP10  	// Short on powerup while JP8 is shorted to also initialize Diode VF

#else
	// Register addresses in the MCP23S17 IO Expander
	#define	IOEXP_IODIRA 0
	#define	IOEXP_IODIRB 1
	#define IOEXP_IOPOLA 2
	#define	IOEXP_IOPOLB 3
	#define	IOEXP_GPINTENA 4
	#define	IOEXP_GPINTENB 5
	#define	IOEXP_DEFVALA 6
	#define	IOEXP_DEFVALB 7
	#define IOEXP_INTCONA 8
	#define	IOEXP_INTCONB 9
	#define	IOEXP_IOCON 10
	#define	IOEXP_GPPUA 12
	#define	IOEXP_GPPUB 13
	#define	IOEXP_INTFA 14
	#define	IOEXP_INTFB 15
	#define	IOEXP_INTCAPA 16
	#define IOEXP_INTCAPB 17
	#define	IOEXP_GPIOA 18
	#define	IOEXP_GPIOB 19
	#define	IOEXP_OLATA 20
	#define	IOEXP_OLATB 21
	
	#define	CTCSSIN	(inputs1 & 0x10)	// Pin 25 - GPA4
	#define	JP8	(inputs1 & 0x40)	// Pin 27 - GPA6 Initialise EEPROM
	#define	JP9	(inputs1 & 0x80)	// Pin 28 - GPA7 Calibrate Squelch
	#define	JP10 	(inputs2 & 1)		// Pin 1  - GPB0 Calibrate Diode
	#define	JP11 	(inputs2 & 2)		// Pin 2  - GPB1 LED 3/4 RX Level Mode

	#define	INITIALIZE (IOExp_Read(IOEXP_GPIOA) & 0x40) 	// Short JP8 on powerup to initialize EEPROM
	#define	INITIALIZE_WVF (IOExp_Read(IOEXP_GPIOB) & 1)  	// Short JP10 on powerup while JP8 is shorted to also initialize Diode VF

#endif // smt

#define WVF 	JP10		// Short to calibrate diode voltage for temperature compensation
#define CAL 	JP9		// Short to calibrate squelch noise. 
				// Shorting the INITIALIZE jumper (JP8) while shorting JP10 also 
				// calibrates the temp. conpensation diode (do at room temp.)
#define	LEVDISP JP11		// Short to change GPS/CONNECT LED's to be audio level display97:

#define SYSLED 	0
#define	SQLED 	1
#define	GPSLED 	2
#define CONNLED 3

#define	BAUD_RATE1 	57600	// Default serial console speed
#define	BAUD_RATE2 	4800	// Default GPS speed

#define	FRAME_SIZE 		160
#define	ADPCM_FRAME_SIZE 	320

#ifdef	DSPBEW
	#define	MAX_BUFLEN 		4800 	// 0.6 seconds of buffer
#else
	#define	MAX_BUFLEN 		6400 	// 0.8 seconds of buffer
#endif

#define	DEFAULT_TX_BUFFER_LENGTH 3000 	// approx 300ms of Buffer
#define	VOTER_CHALLENGE_LEN 	10
#define	ADCOTHERS 		3	// How many "other" ADC channels do we use?
#define	ADCSQNOISE 		0	// Index for squelch noise (RSSI) ADC channel
#define	ADCDIODE 		2	// Index for diode voltage channel
#define	ADCSQPOT 		1	// Index for squelch pot position channel
#define DEFAULT_VOTER_PORT 	1667		// Default UDP port to send on
#define	PPS_WARN_TIME 		(1200 * 8) 	// 1200ms PPS Warning Time
#define PPS_MAX_TIME 		(2400 * 8) 	// 2400 ms PPS Timeout
#define	PPS_MUSTA_TIME 		(950 * 8)
#define	GPS_NMEA_WARN_TIME 	(1200 * 8) 	// 1200 ms GPS Warning Time
#define GPS_NMEA_MAX_TIME 	(2400 * 8) 	// 2400 ms GPS Timeout
#define	GPS_TSIP_WARN_TIME 	(5000ul * 8ul) 	// 5000 ms GPS Warning Time
#define GPS_TSIP_MAX_TIME 	(10000ul * 8ul) // 10000 ms GPS Timeout
#define GPS_FORCE_TIME 		(1500 * 8)  	// Force a GPS (Keepalive) every 1500ms regardless
#define ATTEMPT_TIME 		(500 * 8) 	// Try connection every 500 ms
#define LASTRX_TIME 		(6000ul * 8ul) 	// Timeout if nothing heard after 6 seconds
#define TELNET_TIME 		(100 * 8) 	// Telnet output buffer timer (quasi-Nagle algorithm)
#define	MASTER_TIMING_DELAY 	50 		// Delay to send packet if not master (in 125us increments)
#define BEFORECW_TIME 		(700 * 8) 	// Delay to hold PTT before cw sent 700ms
#define AFTERCW_TIME 		(350 * 8) 	// Delay to hold PTT after cw sent 350ms
#define DSECOND_TIME 		(100 * 8) 	// Delay to hold PTT after cw sent 100ms
#define	MAX_ALT_TIME 		(15 * 2) 	// Seconds to try alt host if not connected
#define	PTT_IGNORE_TIME 	(100 * 8) 	// Ignore PTT for N ms after connection established
#define	MIN_PING_TIME 		(95 * 8) 	// Minimum ping time 95ms
#define	MISS_REPORT_TIME 	100 		// Interval between "miss packet" reports (1/10 secs)
#define	PKT_MISS_TIME 		(500 * 8)	// 500ms for display of miss packet (winky LED) 
#define	SECOND_TIME 		(1000 * 8)	// 1000ms
#define	NCOLS 			75
#define	LEVDISP_FACTOR 		25
#define	TSIP_FACTOR 57.295779513082320876798154814105 // radians to degrees, Trimble reports lat/long in rads
#define ADD_1024_WEEKS 		619315200 	// 1024 weeks for Tbolt time fudge
#define	ULAW_SILENCE 		0xff		// Clamp audio for ulaw silence
#define	ADPCM_SILENCE 		0		// Clamp audio for ADPCM silence
#define	USE_PPS (AppConfig.PPSPolarity != 2)	// 1 if PPS is != ignore
#define	DIAG_WAIT_UART 		(TICK_SECOND / 3ul)	
#define	DIAG_WAIT_MEAS 		(TICK_SECOND * 2)
#define	DIAG_NOISE_GAIN 	0x28
#define	NAPEAKS 		50
#define	QUALCOUNT 		4

#define BIAS 			0x84   		// define the add-in bias for 16 bit ulaw samples
#define CLIP 			32635

#define	DUPLEX3 		(AppConfig.Duplex3 != 0)	// Not supported in voting or simulcast configurations 
#define	SIMULCAST_ENABLE 	(AppConfig.LaunchDelay > 0)	// If the launch delay is anything but 0, use simulcast mode

#ifdef DSPBEW
	#define FFT_BLOCK_LENGTH 	32
	#define LOG2_BLOCK_LENGTH 	5
	#define	FFT_TOP_SAMPLE_BUCKET 	16 // 3000 Hz
	#define	FFT_MAX_RESULT 		10

	fractcomplex sigCmpx[FFT_BLOCK_LENGTH] __attribute__ ((space(ymemory),far,aligned(FFT_BLOCK_LENGTH * 2 *2))); 

#ifndef FFTTWIDCOEFFS_IN_PROGMEM
	fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2] 	/* Declare Twiddle Factor array in X-space*/
	__attribute__ ((section (".xbss, bss, xmemory"), aligned (FFT_BLOCK_LENGTH*2)));
#else
	extern const fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2]	/* Twiddle Factor array in Program memory */
	__attribute__ ((space(auto_psv), aligned (FFT_BLOCK_LENGTH*2)));
#endif // fft

#define ROMNOBEW

#else

#define ROMNOBEW ROM

#endif // DSPBEW

struct meas {
	WORD freq;
	WORD min;
	WORD max;
	BOOL issql;
} ;

enum {GPS_STATE_IDLE,GPS_STATE_RECEIVED,GPS_STATE_VALID,GPS_STATE_SYNCED} ;
enum {GPS_NMEA,GPS_TSIP} ;
enum {CODEC_ULAW,CODEC_ADPCM} ;

// Common IP address format strings
ROM char fmt_ip[] = "%d.%d.%d.%d";
ROM char fmt_ip_newline[] = "%d.%d.%d.%d\n";

// Common menu fragments
ROM char str_enter_newval[] = "Val: ";
ROM char str_enter_sel[] = "Sel: ";

// Common error/status messages
ROM char err_invalid_prefix[] = "Invalid entry: ";
ROM char err_noentry_prefix[] = "No entry: ";
ROM char err_not_changed[] = "Not changed\n";
ROM char msg_changed_success[] = "OK\n";
ROM char msg_error_prefix[] = "ERR: ";

// Consolidated error messages (saves ~60 bytes by using ROM prefix strings)
ROM char err_invalid_notchanged[] = "Inv\n";
ROM char err_noentry_notchanged[] = "None\n";

// Original strings (some now use consolidated strings)
ROM char gpsmsg1[] = "Awaiting lock\n",
		gpsmsg2[] = "Acq, sats=",
		gpsmsg3[] = "Sync OK\n",
		gpsmsg5[] = "Sync lost\n",
		gpsmsg6[] = "Lost, restart\n",
		gpsmsg7[] = "Data TO\n",
		gpsmsg8[] = "PPS TO\n",
		gpsmsg9[] = "Acq\n",
		saved[] = "OK\n",
		invalselection[] = "Inv\n",
		booting[] = "Reboot\n";
 
// These remain in RAM as they may be modified at runtime in some contexts
char 	badmix[] = "MIX rej\n",
 	hosttmomsg[] = "Host TO\n";

// Use inline functions instead of macros to reduce code size
ROM char log_gps_prefix[] = "GPS: ";
ROM char log_net_prefix[] = "NET: ";
ROM char log_rx_prefix[] = "RX: ";
ROM char log_tx_prefix[] = "TX: ";
ROM char log_stat_prefix[] = "STAT: ";
ROM char log_pkt_prefix[] = "PKT: ";
ROM char log_sys_prefix[] = "SYS: ";
ROM char log_err_prefix[] = "ERR: ";
ROM char log_warn_prefix[] = "WARN: ";

// Simplified logging - just prefix with category, caller adds message
#define LOG_GPS(fmt, ...)  do { if (!indisplay) { log_prefix(); printf(log_gps_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
#define LOG_NET(fmt, ...)  do { if (!indisplay) { log_prefix(); printf(log_net_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
/* Radio RX/TX logging uses debug bit 1 */
#define LOG_RX(fmt, ...)   do { if ((!indisplay) && (AppConfig.DebugLevel & 1)) { log_prefix(); printf(log_rx_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
#define LOG_TX(fmt, ...)   do { if ((!indisplay) && (AppConfig.DebugLevel & 1)) { log_prefix(); printf(log_tx_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
/* STAT category for per-second statistics, enabled by debug bit 2
	Note: messages should start with the short category like "TX STAT:" or "RX STAT:" so the final
	output becomes: "[timestamp] TX STAT: ..." */
#define LOG_STAT(fmt, ...)  do { if ((!indisplay) && (AppConfig.DebugLevel & 2)) { log_prefix(); printf(fmt, ##__VA_ARGS__); } } while(0)
#define LOG_PKT(fmt, ...)  do { if (!indisplay) { log_prefix(); printf(log_pkt_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
#define LOG_SYS(fmt, ...)  do { if (!indisplay) { log_prefix(); printf(log_sys_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
#define LOG_ERR(fmt, ...)  do { if (!indisplay) { log_prefix(); printf(log_err_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
#define LOG_WARN(fmt, ...) do { if (!indisplay) { log_prefix(); printf(log_warn_prefix); printf(fmt, ##__VA_ARGS__); } } while(0)
/* GPS Debug logging uses debug bit 32 */
#define LOG_GPS_DEBUG(fmt, ...) do { if ((!indisplay) && (AppConfig.DebugLevel & 32)) { log_prefix(); printf("GPS-DEBUG: "); printf(fmt, ##__VA_ARGS__); } } while(0)

// Common state names for consistency
ROM char log_detected[] = "det";
ROM char log_lost[] = "lost";
ROM char log_open[] = "opn";
ROM char log_closed[] = "cls";
ROM char log_asserted[] = "on";
ROM char log_deasserted[] = "off";
ROM char log_rising[] = "Hi";
ROM char log_falling[] = "Lo";

typedef struct {
	DWORD vtime_sec;
	DWORD vtime_nsec;
} VTIME;

typedef struct {
	VTIME curtime;
	BYTE challenge[VOTER_CHALLENGE_LEN];
	DWORD digest;
	WORD payload_type;
} VOTER_PACKET_HEADER;

VTIME system_time;
VTIME last_rxpacket_time;
VTIME last_rxpacket_sys_time;
long last_rxpacket_index;
char last_rxpacket_inbounds;

// Option flags sent by the host to the client, set in voter.conf for the client
#define OPTION_FLAG_FLATAUDIO 		1	// Send Flat Audio (nodeemp or hostdeemp)
#define	OPTION_FLAG_SENDALWAYS 		2	// Send Audio always (master)
#define OPTION_FLAG_NOCTCSSFILTER 	4 	// Do not filter CTCSS (noplfilter)
#define	OPTION_FLAG_MASTERTIMING 	8  	// Master Timing Source (do not delay sending audio packet) (master)
#define	OPTION_FLAG_ADPCM 		16 	// Use ADPCM rather then ULAW (adpcm)
#define	OPTION_FLAG_MIX 		32 	// Request "Mix" option to host (mixminux)

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;
BYTE AN0String[8];
void SaveAppConfig(void);

/*****************************************************************************/
//									     //
// 	Private helper functions					     //
//									     //
/*****************************************************************************/
// These may or may not be present in all applications.
static void InitAppConfig(void);
static void InitializeBoard(void);

// Squelch RAM variables intended to be read by other modules
extern BOOL cor;		// COR output
extern BOOL sqled;		// Squelch LED output
extern BOOL write_eeprom_cali;	// Flag to write calibration values back to EEPROM
extern BYTE noise_gain;		// Noise gain sent to digital pot
extern WORD caldiode;		// Diode voltage (used for temperature compensation)

void service_squelch(WORD diode,WORD sqpos,WORD noise,BOOL cal,BOOL wvf,BOOL iscaled);
void init_squelch(void);
BOOL set_atten(BYTE val);

// Forward declaration for SetConnStatus - defined later, but macro needed early for SMT boards
#if defined(SMT_BOARD)
#define SetConnStatus(val)  do { } while(0)  // No-op for SMT boards
#else
static inline void SetConnStatus(BOOL val);  // Forward declaration for non-SMT boards
#endif

/*****************************************************************************/
//									     //
//	Global Variable Definitions					     //
//									     //
/*****************************************************************************/
WORD portasave;		// Get the PPS input from RA4(CN0) on interrupt
BYTE inputs1;		// GPA I/O on IO Expander
BYTE inputs2;		// GPB I/O on IO Expander
BYTE filling_buffer;
WORD fillindex;
BOOL filled;
BYTE audio_buf[2][FRAME_SIZE + 3];	// Audio buffer array
BOOL connected;		// Connected to host
BYTE rssi;		
BYTE rssiheld;		
BYTE gps_buf[160];	// GPS receive buffer array
BYTE gps_bufindex;	// GPS receive buffer array index pointer
BYTE TSIPwasdle;
BYTE gps_state;		// Current GPS state (idle, receiving, valid, synched) 
BYTE gps_nsat;		// Number of satellites in view (not necessarily locked)
BOOL gpssync;		// Set only when GPS_STATE_VALID
BOOL gotpps;		// Set only when GPS_STATE_VALID and PPS is valid
DWORD gps_time;		// GPS time in seconds
WORD gpsweek;		// GPS week reported by TSIP devices
WORD gpsleap;		// GPS leap seconds reported by TSIP devices for correcting to UTC time
BYTE ppscount;
DWORD last_interval;
BYTE lockcnt;
BOOL adcother;		// Flag for whether to encode an RX packet, or measure other ADC channels
WORD adcothers[ADCOTHERS];	// Array for holding the "other" ADC values (RX Noise, Sq Pot, Diode V)
BYTE adcindex;		// Counter for measuring the "other" ADC channels
BYTE sqlcount;		// Counter for how often we service squelch and RSSI (set to 33 below, so every 33 ADC sample periods)
BOOL sql2;
DWORD vnoise32;
DWORD lastvnoise32[3];
BOOL wascor;
BOOL lastcor;
BYTE option_flags;	// Holds the option flags we get from the host
static struct {
	VOTER_PACKET_HEADER vph;
	BYTE rssi;
	BYTE audio[FRAME_SIZE + 3];
} audio_packet;
static struct {
	VOTER_PACKET_HEADER vph;
	char lat[9];
	char lon[10];
	char elev[7];
} gps_packet;
WORD txdrainindex;
WORD last_drainindex;
VTIME lastrxtime;
BOOL ptt;
BOOL host_ptt;
DWORD gpstimer;
WORD ppstimer;
WORD gpsforcetimer;
WORD attempttimer;
DWORD lastrxtimer;
WORD cwtimer;
BYTE gpswarn;
BOOL ppswarn;
BOOL ppsx;
UDP_SOCKET udpSocketUser;
NODE_INFO udpServerNode;
DWORD dwLastIP;
BOOL aborted;
BOOL inread;
IP_ADDR MyVoterAddr;
IP_ADDR LastVoterAddr;
IP_ADDR MyAltVoterAddr;
IP_ADDR LastAltVoterAddr;
IP_ADDR CurVoterAddr;
WORD dnstimer;
BOOL dnsdone;
DWORD timing_time;
WORD timing_index;
DWORD next_time;
DWORD next_index;
WORD samplecnt;
WORD last_adcsample;
WORD last_index;
WORD last_index1;
DWORD real_time;
WORD last_samplecnt;
DWORD digest;
DWORD resp_digest;
DWORD mydigest;
BOOL sendgps;
BYTE dnsnotify;
BYTE altdnsnotify;
long discfactor;
long discounterl;
long discounteru;
short amax;
short amin;
WORD apeak;
BOOL indisplay;
BOOL indipsw;
short enc_valprev;	/* Previous output value */
char enc_index;		/* Index into stepsize table */
short enc_prev_valprev;	/* Previous output value */
char enc_prev_index;	/* Index into stepsize table */
BYTE enc_lastdelta;
BYTE dec_buffer[FRAME_SIZE * 2];
short dec_valprev;	/* Previous output value */
char dec_index;		/* Index into stepsize table */
BOOL time_filled;
long host_txseqno;
long txseqno_ptt;
long txseqno;
DWORD elketimer;
WORD testidx;
short *testp;
BYTE leddiag;
BYTE diagstate;
BYTE measretstate;
BYTE errcnt;
struct meas *measp;
BYTE measidx;
char *measstr;
BYTE diag_option_flags;
WORD apeaks[NAPEAKS];
BOOL netisup;
BOOL gotbadmix;
BOOL telnet_echo;
WORD termbufidx;
WORD termbuftimer;
BYTE linkstate;
BYTE cwlen;
BYTE cwdat;
char *cwptr;
WORD cwtimer;
BYTE cwidx;
WORD cwtimer1;
WORD cwid_ptt_delay_timer;
BOOL connrep;
BYTE connfail;
BOOL ever_connected;
WORD failtimer;
WORD hangtimer;
WORD dsecondtimer;
BOOL repeatit;
BOOL needburp;
char their_challenge[VOTER_CHALLENGE_LEN];
char challenge[VOTER_CHALLENGE_LEN];
char cmdstr[50];
BYTE txaudio[MAX_BUFLEN];
long tone_v1;
long tone_v2;
long tone_v3;
long tone_fac;
BOOL hosttimedout;
BOOL altdns;
BOOL althost;
BOOL lastalthost;
BOOL altconnected;
WORD alttimer;
WORD ptt_ignore_timer;
BOOL altchange;
BOOL altchange1;
WORD glasertimer;
WORD conn_status_offline_timer;
DWORD uptimer;
WORD pingtimer;
WORD secondtimer;
long missed;
WORD misstimer;
WORD misstimer1;
WORD saved_rcon;
DWORD cold_power_reboot_target;
BOOL cold_power_reboot_active;

#ifdef DSPBEW
	DWORD fftresult;
#endif

BYTE myDHCPBindCount;

#if !defined(STACK_USE_DHCP)
     //If DHCP is not enabled, force DHCP update.
    #define DHCPBindCount 1
#endif

// Compact CRC-32 implementation (polynomial 0xEDB88320) to avoid 1KB ROM table
static inline unsigned long crc32_update(unsigned long crc, unsigned char data)
{
	unsigned long c = crc ^ data;
	int i;
	for (i = 0; i < 8; i++)
	{
		if (c & 1)
			c = (c >> 1) ^ 0xEDB88320UL;
		else
			c >>= 1;
	}
	return c;
}

static long crc32_bufs(unsigned char *buf, unsigned char *buf1)
{
	unsigned long crc = 0xFFFFFFFFUL;
	
	while (buf && *buf)
	{
		crc = crc32_update(crc, *buf++);
	}
	while (buf1 && *buf1)
	{
		crc = crc32_update(crc, *buf1++);
	}
	return (long)(~crc);
}

ROM BYTE exp_lut[256] = {
	0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,
	4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
	5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7 };


ROM short ulawtabletx[] = {
//-32124,-31100,-30076,-29052,-28028,-27004,-25980,-24956,
-12000,-12000,-12000,-12000,-12000,-27004,-25980,-24956,
-23932,-22908,-21884,-20860,-19836,-18812,-17788,-16764,
-15996,-15484,-14972,-14460,-13948,-13436,-12924,-12412,
-11900,-11388,-10876,-10364,-9852,-9340,-8828,-8316,
-7932,-7676,-7420,-7164,-6908,-6652,-6396,-6140,
-5884,-5628,-5372,-5116,-4860,-4604,-4348,-4092,
-3900,-3772,-3644,-3516,-3388,-3260,-3132,-3004,
-2876,-2748,-2620,-2492,-2364,-2236,-2108,-1980,
-1884,-1820,-1756,-1692,-1628,-1564,-1500,-1436,
-1372,-1308,-1244,-1180,-1116,-1052,-988,-924,
-876,-844,-812,-780,-748,-716,-684,-652,
-620,-588,-556,-524,-492,-460,-428,-396,
-372,-356,-340,-324,-308,-292,-276,-260,
-244,-228,-212,-196,-180,-164,-148,-132,
-120,-112,-104,-96,-88,-80,-72,-64,
-56,-48,-40,-32,-24,-16,-8,0,
//32124,31100,30076,29052,28028,27004,25980,24956,
12000,12000,12000,12000,12000,27004,25980,24956,
23932,22908,21884,20860,19836,18812,17788,16764,
15996,15484,14972,14460,13948,13436,12924,12412,
11900,11388,10876,10364,9852,9340,8828,8316,
7932,7676,7420,7164,6908,6652,6396,6140,
5884,5628,5372,5116,4860,4604,4348,4092,
3900,3772,3644,3516,3388,3260,3132,3004,
2876,2748,2620,2492,2364,2236,2108,1980,
1884,1820,1756,1692,1628,1564,1500,1436,
1372,1308,1244,1180,1116,1052,988,924,
876,844,812,780,748,716,684,652,
620,588,556,524,492,460,428,396,
372,356,340,324,308,292,276,260,
244,228,212,196,180,164,148,132,
120,112,104,96,88,80,72,64,
56,48,40,32,24,16,8,0
};

/* Intel ADPCM step variation table */
ROM static int indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

ROM static int stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

#define CWTONELEN 10

static ROM short cwtone[CWTONELEN] = {
    0,  2407, 3895, 3895,  2407, 0, -2407, -3895, -3895, -2407
};

struct morse_bits
{
    BYTE len;
    BYTE dat;
};

static ROM struct morse_bits mbits[] = {
    {0, 0}, /* SPACE */
    {0, 0},
    {6, 18},/* " */
    {0, 0},
    {7, 72},/* $ */
    {0, 0},
    {0, 0},
    {6, 30},/* ' */
    {5, 13},/* ( */
    {6, 29},/* ) */
    {0, 0},
    {5, 10},/* + */
    {6, 51},/* , */
    {6, 33},/* - */
    {6, 42},/* . */
    {5, 9}, /* / */
    {5, 31},/* 0 */
    {5, 30},/* 1 */
    {5, 28},/* 2 */
    {5, 24},/* 3 */
    {5, 16},/* 4 */
    {5, 0}, /* 5 */
    {5, 1}, /* 6 */
    {5, 3}, /* 7 */
    {5, 7}, /* 8 */
    {5, 15},/* 9 */
    {6, 7}, /* : */
    {6, 21},/* ; */
    {0, 0},
    {5, 33},/* = */
    {0, 0},
    {6, 12},/* ? */
    {0, 0},
    {2, 2}, /* A */
    {4, 1}, /* B */
    {4, 5}, /* C */
    {3, 1}, /* D */
    {1, 0}, /* E */
    {4, 4}, /* F */
    {3, 3}, /* G */
    {4, 0}, /* H */
    {2, 0}, /* I */
    {4, 14},/* J */
    {3, 5}, /* K */
    {4, 2}, /* L */
    {2, 3}, /* M */
    {2, 1}, /* N */
    {3, 7}, /* O */
    {4, 6}, /* P */
    {4, 11},/* Q */
    {3, 2}, /* R */
    {3, 0}, /* S */
    {1, 1}, /* T */
    {3, 4}, /* U */
    {4, 8}, /* V */
    {3, 6}, /* W */
    {4, 9}, /* X */
    {4, 13},/* Y */
    {4, 3}  /* Z */
};

static ROM char paktc[] = "\nPress Enter...\n";

char dummy_loc;
BYTE IOExpOutA,IOExpOutB,IODirB;

void main_processing_loop(void);

static WORD log2fix (WORD x)
{
	short b = 127;
	short y = 0;
	BYTE i;

	while (x < 256) 
	{
		x <<= 1;
		y -= 256;
	}

	while (x >= 512) 
	{
		x >>= 1;
		y += 256;
	}

	DWORD z = x;

	for (i = 0; i < 8; i++) 
	{
		z = z * z >> 8;
		if (z >= 512) 
		{
			z >>= 1;
			y += b;
		}
		b >>= 1;
	}

	return y;
}

// Integer square root. Approximates the square root of a number, integer part 
// only, no rounding. ie 12.96 will return 12
static DWORD isqrt(DWORD number)
{
        if(number <= 3) { return number > 0; }

        DWORD oldAns = number >> 1,                     // initial guess
            newAns = (oldAns + number / oldAns) >> 1; 	// first iteration

        // main iterative method
        while(newAns < oldAns)
        {
                oldAns = newAns;
                newAns = (oldAns + number / oldAns) >> 1;
        }

        return oldAns;
}

static BYTE calcrssi(WORD val)
{
DWORD d;
short i,x;

	if (val < 200) // The first part of this IF is "Chuck RSSI"
	{
	        x = 255 - val;
	}
	else
	{ // This is the original RSSI calculation
	        d = (((DWORD)val + 1) << 8);
	        i = log2fix(isqrt(d) << 4);
	        x = 255 - ((i << 3) / 39);
	        if (x < 0) x = 0;
	}
	return(x);
}

/* This ISR uses Change Notification, and runs when PPS changes (each PPS tick). PPS is connected to RA4 (CN0).
   Every time we get a PPS signal, we mask off RA4 (0x10) and read it. If it is valid, we clear ppsx and continue. */ 
void __attribute__((auto_psv,__interrupt__(__preprologue__("push W7\n\tmov PORTA,w7\n\tmov W7,_portasave\n\tpop W7")))) _CNInterrupt(void)
{
	//Stuff for ADPCM encode
	int val;			/* Current input sample value */
	int sign;			/* Current adpcm sign bit */
	BYTE delta;			/* Current adpcm output value */
	int diff;			/* Difference between val and valprev */
	int step;			/* Stepsize */
	int vpdiff;			/* Current change to valpred */
	long valpred;		/* Predicted output value */
	int adpcm_index;
	BYTE *cp;

	static BOOL pps_polarity_warning_shown = 0;
	static BOOL last_gotpps_logged = 0;

	CORCONbits.PSV = 1;
	// If PPS signal is asserted
	if (((portasave & 0x10) && (AppConfig.PPSPolarity == 0)) ||
		((!(portasave & 0x10)) && (AppConfig.PPSPolarity == 1))) 
		ppsx = 1; // PPS not good
	else
		ppsx = 0; // PPS is good

	if (ppsx || (ppstimer >= PPS_MUSTA_TIME))
	{
		if (USE_PPS)
		{
			ppstimer = 0;
			ppswarn = 0;
			pps_polarity_warning_shown = 0; // Reset warning flag on good PPS
			// If GPS is now VALID, but we haven't qualified pps yet, do it
			if ((gps_state == GPS_STATE_VALID) && (!gotpps))
			{
				TMR3 = 0;	// Reset the Timer 3 register
				gotpps = 1;	// GPS is good, so PPS must be good
				last_gotpps_logged = 1;
				lockcnt = 0;
				samplecnt = 0;
				fillindex = 0;
			}
			else if (gotpps) 	// PPS is already qualified
			{
				if (ppscount >= 3)
				{	// Setup Timer 4 for Internal Clock (Fosc/2 aka Fcy)
					// Divide/8 prescaler
					// Therefore, each tick is 1/(38.4MHz/8) = 0.2083uS
					// When this timer expires (after launch delay), it will call the 
					// T4 interrupt ISR, turning on the DAC for TX
					T4CON = 0x10;
					PR4 = AppConfig.LaunchDelay + 5; // Give 1us to let it get outa the CN interrupt!!!! :-)		
					TMR4 = 0;		// Reset the timer register
					IFS1bits.T4IF = 0;	// Clear the timer flag
					IEC1bits.T4IE = 1;	// Enable interrupts
					IPC6bits.T4IP = 6;	// Set interrupt priority 6
					T4CONbits.TON = 1;	// Turn timer on
	
					if ((samplecnt >= 7999) && (samplecnt <= 8001))
					{
						last_samplecnt = samplecnt;
						sendgps = 1;
						real_time++;

						if ((samplecnt < 8000) && (!SIMULCAST_ENABLE)) // If we are short one, insert another
						{
							if (option_flags & OPTION_FLAG_ADPCM)
							{
								val = last_adcsample;
								val -= 2048;
								val *= 16;
								adpcm_index = enc_index;
								valpred = enc_valprev;
								step = stepsizeTable[adpcm_index];
								
								/* Step 1 - compute difference with previous value */
								diff = val - valpred;
								sign = (diff < 0) ? 8 : 0;

								if ( sign ) 
									diff = (-diff);
							
								/* Step 2 - Divide and clamp */
								/* Note:
								** This code *approximately* computes:
								**    delta = diff*4/step;
								**    vpdiff = (delta+0.5)*step/4;
								** but in shift step bits are dropped. The net result of this is
								** that even if you have fast mul/div hardware you cannot put it to
								** good use since the fixup would be too expensive.
								*/
								delta = 0;
								vpdiff = (step >> 3);
								
								if ( diff >= step ) 
								{
									delta = 4;
									diff -= step;
									vpdiff += step;
								}
								
								step >>= 1;
								
								if ( diff >= step  ) 
								{
									delta |= 2;
									diff -= step;
									vpdiff += step;
								}
								
								step >>= 1;
								
								if ( diff >= step ) 
								{
									delta |= 1;
									vpdiff += step;
								}
							
								/* Step 3 - Update previous value */
								if ( sign )
									valpred -= vpdiff;
								else
									valpred += vpdiff;
							
								/* Step 4 - Clamp previous value to 16 bits */
								if ( valpred > 32767 )
									valpred = 32767;
								else if ( valpred < -32768 )
									valpred = -32768;
							
								/* Step 5 - Assemble value, update index and step values */
								delta |= sign;
								
								adpcm_index += indexTable[delta];
								
								if ( adpcm_index < 0 ) 
									adpcm_index = 0;
								
								if ( adpcm_index > 88 ) 
									adpcm_index = 88;
								enc_valprev = valpred;
								enc_index = adpcm_index;
								
								if (fillindex & 1)
								{
									audio_buf[filling_buffer][fillindex++ >> 1] = (enc_lastdelta << 4) | delta;
								}
								else
								{
									enc_lastdelta = delta;
								}
							}
							else  // is ULAW
							{
						        	short sample,sign, exponent, mantissa;
						        	BYTE ulawbyte;
								
								sample = last_adcsample;
								sample -= 2048;
								sample *= 16;

						        	/* Get the sample into sign-magnitude. */
								sign = (sample >> 8) & 0x80;	/* set aside the sign */
								
								if (sign != 0)
									sample = -sample;	/* get magnitude */
								
							        if (sample > CLIP)
									sample = CLIP;		/* clip the magnitude */
						
								/* Convert from 16 bit linear to ulaw. */
								sample = sample + BIAS;
								exponent = exp_lut[(sample >> 7) & 0xFF];
								mantissa = (sample >> (exponent + 3)) & 0x0F;
								ulawbyte = ~(sign | (exponent << 4) | mantissa);
								audio_buf[filling_buffer][fillindex++] = ulawbyte;
							}

							if (fillindex >= ((option_flags & OPTION_FLAG_ADPCM) ? FRAME_SIZE * 2 : FRAME_SIZE))
							{
								if (option_flags & OPTION_FLAG_ADPCM)
								{
									cp = &audio_buf[filling_buffer][fillindex >> 1];
									*cp++ = (enc_prev_valprev & 0xff00) >> 8;
									*cp++ = enc_prev_valprev & 0xff;
									*cp = enc_index;
									enc_prev_valprev = enc_valprev;
									enc_prev_index = enc_index;
								}

								filled = 1;
								fillindex = 0;
								filling_buffer ^= 1;
								last_drainindex = txdrainindex;
								timing_index = next_index;
								timing_time = next_time;
							}
						}

						if ((!gpssync) && (gps_state == GPS_STATE_VALID))
						{
							system_time.vtime_sec = timing_time = real_time = gps_time + 1; 
							gpssync = 1;
						}
					}
					else
					{
						gpssync = 0;
						ppscount = 0;
					}
				} 
				else ppscount++;
			}
			samplecnt = 0;
		}
	}
	IFS1bits.CNIF = 0;	// Clear the CN interript flag, we're done!
}

/*****************************************************************************/
//                                                                           //
//              T4 Interrupt ISR                                             //
//                                                                           //
/*****************************************************************************/
/* This ISR is called when the Launch Delay (+1uS) expires.
   It turns on the DAC for TX output, after the launch delay
*/
void __attribute__((interrupt, auto_psv)) _T4Interrupt(void)
{
	CORCONbits.PSV = 1;
	IFS1bits.T4IF = 0;	// Clear the T4 interrupt flag
	IEC1bits.T4IE = 0;	// Disable the T4 interrupts
	T4CONbits.TON = 0;	// Turn off the T4 timer
	DAC1CONbits.DACEN = 1;	// Enable DAC1
	IEC4bits.DAC1LIE = 1;	// Enable DAC1 Left interrupt
}

/*****************************************************************************/
//									     //
//		ADC ISR							     //
//									     //
/*****************************************************************************/
/* Every time TMR3 expires (62.5uSec), we service the ADC.
   On ODD calls of this ISR, we grab an RX Audio value and encode it, 
   which means, every 125uSec we encode a packet, or 8000 samples/sec (8kHz) 
   audio.
   On EVEN calls of this ISR, we rotate between getting values for RXNoise 
   (RSSI), Squelch Pot position, and Diode Voltage (temp comp).
   EVERY time through, we bump some counters.
*/ 
void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void)
{
	WORD index;		// Current ADC Buffer 12-bit unsigned value (0x0000 to 0x0fff)

	long accum;
	short saccum;
	BYTE i;

	//Stuff for ADPCM encode
	int val;		/* Current input sample value */
	int sign;		/* Current adpcm sign bit */
	BYTE delta;		/* Current adpcm output value */
	int diff;		/* Difference between val and valprev */
	int step;		/* Stepsize */
	int vpdiff;		/* Current change to valpred */
	long valpred;		/* Predicted output value */
	int adpcm_index;
	BYTE *cp;

	CORCONbits.PSV = 1;
	index = ADC1BUF0;	// Copy the current ADC buffer value

	if (adcother) 	// True if this time we're processing other ADC channels (not RX Audio)
	{
		// Divide by 4, effectively scaling the ADC value to 0x000-0x3ff (0-1023)
		// and increment the ADC channel index (used at the end to select the next channel)
		adcothers[adcindex++] = index >> 2;
		if(adcindex >= ADCOTHERS) adcindex = 0; // Reset if we've already gone through all the channels
		AD1CHS0 = 0; // Reset our ADC channel (0 is RX Audio)

		// Bump some timers to make sure everything is okay
		if (gotpps) ppstimer++;
		if (gps_state != GPS_STATE_IDLE) gpstimer++;

	if (connected) 	// If we're connected to the host, update some timers
	{
		gpsforcetimer++;
		elketimer++;
		if (glasertimer) glasertimer--;
		if (pingtimer) pingtimer--;
		if (misstimer1) misstimer1--;
		if (ptt_ignore_timer < PTT_IGNORE_TIME) ptt_ignore_timer++;
	}

	if (!connected) attempttimer++;
		else lastrxtimer++;

		termbuftimer++;

		if (cwtimer1) cwtimer1--;
			dsecondtimer++;

		if (dsecondtimer >= DSECOND_TIME)
		{
			dsecondtimer = 0;
			if (hangtimer) hangtimer--;
			failtimer++;
			uptimer++;
			if (misstimer) misstimer--;
		}

		if (secondtimer++ >= SECOND_TIME)
		{
			secondtimer = 0;
		}
	}
	else	// Not processing other ADC channels, we're doing RX Audio
	{
		last_index = last_index1;	// Previous sample becomes last_index
		last_index1 = index;		// Current sample becomes last_index1
	// If we're not simulcasting, or we are simulcasting and not using PPS, 
	// we're going to encode an RX Audio sample. 
	// Otherwise, we're just going to skip it.
		if (!(SIMULCAST_ENABLE && USE_PPS))
		{
			if (gotpps || (!USE_PPS))
			{
				if (fillindex == 0)
				{
					next_index = samplecnt;
					next_time = real_time;
				}
				
				last_adcsample = index;	// index is the current ADC Buffer value
				
				// Make 16 bit number from 12 bit ADC sample
				saccum = index;	// index is the current ADC Buffer value
				saccum -= 2048;
				accum = saccum * 16;

	            		if (accum > amax)
	            		{
	                		amax = accum;
	                		discounteru = discfactor;
	            		}
	            		else if (--discounteru <= 0)
	            		{
	                		discounteru = discfactor;
	                		amax = (long)((amax * 32700) / 32768L);
	            		}

				if (accum < amin)
	            		{
	                		amin = accum;
	                		discounterl = discfactor;
	            		}
	            		else if (--discounterl <= 0)
	            		{
	                		discounterl = discfactor;
	                		amin = (long)((amin * 32700) / 32768L);
				}
				
				// Reset the sample counter when we hit 8000 samples
				if ((!USE_PPS) && (samplecnt == 8000)) samplecnt = 0;
	
				if (samplecnt++ < 8000)
				{
					if (option_flags & OPTION_FLAG_ADPCM)
					{
						val = index;
						val -= 2048;
						val *= 16;
					
						adpcm_index = enc_index;
						valpred = enc_valprev;
						step = stepsizeTable[adpcm_index];
							
						/* Step 1 - compute difference with previous value */
						diff = val - valpred;
						sign = (diff < 0) ? 8 : 0;

						if ( sign ) diff = (-diff);
						
						/* Step 2 - Divide and clamp */
						/* Note:
						** This code *approximately* computes:
						**    delta = diff*4/step;
						**    vpdiff = (delta+0.5)*step/4;
						** but in shift step bits are dropped. The net result of this is
						** that even if you have fast mul/div hardware you cannot put it to
						** good use since the fixup would be too expensive.
						*/
						delta = 0;
						vpdiff = (step >> 3);
						
						if ( diff >= step ) 
						{
							delta = 4;
							diff -= step;
							vpdiff += step;
						}

						step >>= 1;

						if ( diff >= step  ) 
						{
							delta |= 2;
							diff -= step;
							vpdiff += step;
						}

						step >>= 1;

						if ( diff >= step ) 
						{
							delta |= 1;
							vpdiff += step;
						}
						
						/* Step 3 - Update previous value */
						if ( sign )
							valpred -= vpdiff;
						else
							valpred += vpdiff;
						
						/* Step 4 - Clamp previous value to 16 bits */
						if ( valpred > 32767 )
							valpred = 32767;
						else if ( valpred < -32768 )
							valpred = -32768;
						
						/* Step 5 - Assemble value, update index and step values */
						delta |= sign;
						
						adpcm_index += indexTable[delta];

						if ( adpcm_index < 0 ) 
							adpcm_index = 0;

						if ( adpcm_index > 88 ) 
							adpcm_index = 88;
						enc_valprev = valpred;
						enc_index = adpcm_index;

						if (fillindex & 1)
						{
							audio_buf[filling_buffer][fillindex >> 1]	= (enc_lastdelta << 4) | delta;
						}
						else
						{
							enc_lastdelta = delta;
						}
						fillindex++;
					}
					else  // is ULAW
					{
						short sample,sign, exponent, mantissa;
						BYTE ulawbyte;
					
						sample = index;
						sample -= 2048;
						sample *= 16;
					
					        /* Get the sample into sign-magnitude. */
					        sign = (sample >> 8) & 0x80;	/* set aside the sign */

					        if (sign != 0)
					                sample = -sample;	/* get magnitude */

					        if (sample > CLIP)
					                sample = CLIP;		/* clip the magnitude */
					
					        /* Convert from 16 bit linear to ulaw. */
					        sample = sample + BIAS;
					        exponent = exp_lut[(sample >> 7) & 0xFF];
					        mantissa = (sample >> (exponent + 3)) & 0x0F;
					        ulawbyte = ~(sign | (exponent << 4) | mantissa);
					
							audio_buf[filling_buffer][fillindex++] = ulawbyte;
					}
					
					if (txseqno == 0) txseqno = 3;
					
					if (fillindex >= ((option_flags & OPTION_FLAG_ADPCM) ? FRAME_SIZE * 2 : FRAME_SIZE))
					{
						if (option_flags & OPTION_FLAG_ADPCM)
						{
							cp = &audio_buf[filling_buffer][fillindex >> 1];
							*cp++ = (enc_prev_valprev & 0xff00) >> 8;
							*cp++ = enc_prev_valprev & 0xff;
							*cp = enc_prev_index;
							enc_prev_valprev = enc_valprev;
							enc_prev_index = enc_index;
							txseqno++;
							
							if (host_txseqno) host_txseqno++;
						}
						
						txseqno++;
							
						if (host_txseqno) host_txseqno++;
							
						filled = 1;
						fillindex = 0;
						filling_buffer ^= 1;
						last_drainindex = txdrainindex;
						timing_index = next_index;
						timing_time = next_time;
						apeak = (long)(amax - amin) / 2;
						for(i = NAPEAKS -1; i; i--) apeaks[i] = apeaks[i - 1];
						apeaks[0] = apeak;
					}
				}
			}
		}
#if defined(SMT_BOARD)
		AD1CHS0 = adcindex + 1; // Select the next non-RX ADC channel for next time
#else
		AD1CHS0 = adcindex + 2; // Select the next non-RX ADC channel for next time
#endif
		sqlcount++;
	}
	adcother ^= 1;		// Toggle adcother so we switch between doing an RX sample or other ADC sample
	IFS0bits.AD1IF = 0; 	// Clear the interrupt flag, we're done!
}
/***************End of ADC ISR************************************************/

/*****************************************************************************/
//									     //
//		DAC ISR							     //
//									     //
/*****************************************************************************/
/* In order for this ISR to run, the Launch Delay timer must expire, 
   so that the interrupt gets enabled.
*/
void __attribute__((interrupt, auto_psv)) _DAC1LInterrupt(void)
{
	BYTE c;
	short s;

	WORD index;
	long accum;
	short saccum;
	BYTE i;

	//Stuff for ADPCM encode
	int val;		// Current input sample value 
	int sign;		// Current adpcm sign bit 
	BYTE delta;		// Current adpcm output value 
	int diff;		// Difference between val and valprev 
	int step;		// Stepsize 
	int vpdiff;		// Current change to valpred
	long valpred;		// Predicted output value 
	int adpcm_index;
	BYTE *cp;

	CORCONbits.PSV = 1;
	IFS4bits.DAC1LIF = 0;	// Clear the DAC1Left Interrupt Flag
	s = 0;
	// Output Tx sample
	if (testp)
	{
		DAC1LDAT = testp[testidx++ + 1];
		if (testidx >= testp[0]) testidx = 0;
	}
	else 
	{
		if (cwptr && (!cwtimer1))
		{
			if (--cwtimer)
			{
				if ((cwlen > 1) && (!(cwlen & 1))) s = cwtone[cwidx++];
				if (cwidx >= CWTONELEN) cwidx = 0;
			}
			else
			{
				cwidx = 0;
				if (cwlen)
				{
					cwlen--;
					cwtimer = AppConfig.CWSpeed;
					if (!(cwlen & 1))
					{
						if (cwlen > 1)
						{
							if (cwdat & 1) cwtimer = AppConfig.CWSpeed * 3;
							cwdat = cwdat >> 1;
						}
					}
					else if (cwlen == 1) 
						cwtimer = AppConfig.CWSpeed * 2;
				}
				else
				{
					cwtimer = 0;
					cwlen = 0;
					cwdat = 0;
					while((c = *cwptr++))
					{
						if (c < ' ') continue;
						if (c > 'Z') continue;
						c -= ' ';
						if (mbits[c].len)
						{
							cwlen = mbits[c].len * 2;
							cwdat = mbits[c].dat;
							if (cwdat & 1) cwtimer = AppConfig.CWSpeed * 3;
							else cwtimer = AppConfig.CWSpeed;
							cwdat = cwdat >> 1;
						}
						else
						{
							cwlen = 1;
							cwdat = 0;
							cwtimer = AppConfig.CWSpeed * 6;
						}
						break;
					}
					if (!cwlen) 
					{
						cwptr = 0;
						cwtimer1 = AppConfig.CWAfterTime;
					}
				}
			}
		}

		if (repeatit)
		{
			short s1;

			s1 = last_index << 4;
			s += s1 - 32768;
		}

		if (ptt && (!host_ptt) && tone_fac)
		{  
		      	tone_v1 = tone_v2;
		        tone_v2 = tone_v3;
	        	tone_v3 = (tone_fac * tone_v2 >> 15) - tone_v1;
			s += tone_v3;
		}

		if (ptt)
		{
			c = txaudio[txdrainindex];

			if (connected)
				DAC1LDAT = ulawtabletx[c] + s;
			else
				DAC1LDAT = s;
		} 
		else DAC1LDAT = 0;

		txaudio[txdrainindex++] = ULAW_SILENCE;

		if (txdrainindex >= AppConfig.TxBufferLength)
		txdrainindex = 0;
	}

	if (SIMULCAST_ENABLE && USE_PPS)
	{
		index = last_index1;

		if (gotpps || (!USE_PPS))
		{
			if (fillindex == 0)
			{
				next_index = samplecnt;
				next_time = real_time;
			}

			last_adcsample = index;
	
			// Make 16 bit number from 12 bit ADC sample
			saccum = index;
			saccum -= 2048;
			accum = saccum * 16;
			
			if (accum > amax)
			{
				amax = accum;
				discounteru = discfactor;
			}
			else if (--discounteru <= 0)
			{
				discounteru = discfactor;
				amax = (long)((amax * 32700) / 32768L);
			}
			
			if (accum < amin)
			{
				amin = accum;
				discounterl = discfactor;
			}
			else if (--discounterl <= 0)
			{
				discounterl = discfactor;
				amin = (long)((amin * 32700) / 32768L);
			}

			if ((!USE_PPS) && (samplecnt == 8000)) samplecnt = 0;
		
			if (samplecnt++ < 8000)
			{
				if (option_flags & OPTION_FLAG_ADPCM)
				{
					val = index;
					val -= 2048;
					val *= 16;
					
					adpcm_index = enc_index;
					valpred = enc_valprev;
					step = stepsizeTable[adpcm_index];
					
					/* Step 1 - compute difference with previous value */
					diff = val - valpred;
					sign = (diff < 0) ? 8 : 0;
				
					if ( sign ) diff = (-diff);
				
					/* Step 2 - Divide and clamp */
					/* Note:
					** This code *approximately* computes:
					**    delta = diff*4/step;
					**    vpdiff = (delta+0.5)*step/4;
					** but in shift step bits are dropped. The net result of this is
					** that even if you have fast mul/div hardware you cannot put it to
					** good use since the fixup would be too expensive.
					*/
					delta = 0;
					vpdiff = (step >> 3);
						
					if ( diff >= step ) 
					{
						delta = 4;
						diff -= step;
						vpdiff += step;
					}
					
					step >>= 1;
					
					if ( diff >= step  ) 
					{
						delta |= 2;
						diff -= step;
						vpdiff += step;
					}
					
					step >>= 1;
					
					if ( diff >= step ) 
					{
						delta |= 1;
						vpdiff += step;
					}
					
					/* Step 3 - Update previous value */
					if ( sign )
						valpred -= vpdiff;
					else
						valpred += vpdiff;
					
					/* Step 4 - Clamp previous value to 16 bits */
					if ( valpred > 32767 )
						valpred = 32767;
					else if ( valpred < -32768 )
						valpred = -32768;
				
					/* Step 5 - Assemble value, update index and step values */
					delta |= sign;
					adpcm_index += indexTable[delta];
					
					if ( adpcm_index < 0 ) 
						adpcm_index = 0;
						
					if ( adpcm_index > 88 ) 
						adpcm_index = 88;
				
					enc_valprev = valpred;
					enc_index = adpcm_index;
						
					if (fillindex & 1)
					{
						audio_buf[filling_buffer][fillindex >> 1]	= (enc_lastdelta << 4) | delta;
					}
					else
					{
						enc_lastdelta = delta;
					}

					fillindex++;
				}
				else  // is ULAW
				{
					short sample,sign, exponent, mantissa;
					BYTE ulawbyte;
				
					sample = index;
					sample -= 2048;
					sample *= 16;
	
				        /* Get the sample into sign-magnitude. */
				        sign = (sample >> 8) & 0x80;	/* set aside the sign */
				        if (sign != 0)
			        	        sample = -sample;	/* get magnitude */
					
					if (sample > CLIP)
						sample = CLIP;		/* clip the magnitude */
			
					/* Convert from 16 bit linear to ulaw. */
					sample = sample + BIAS;
					exponent = exp_lut[(sample >> 7) & 0xFF];
					mantissa = (sample >> (exponent + 3)) & 0x0F;
					ulawbyte = ~(sign | (exponent << 4) | mantissa);
					audio_buf[filling_buffer][fillindex++] = ulawbyte;
				}
				
				if (txseqno == 0) txseqno = 3;

				if (fillindex >= ((option_flags & OPTION_FLAG_ADPCM) ? FRAME_SIZE * 2 : FRAME_SIZE))
				{
					if (option_flags & OPTION_FLAG_ADPCM)
					{
						cp = &audio_buf[filling_buffer][fillindex >> 1];
						*cp++ = (enc_prev_valprev & 0xff00) >> 8;
						*cp++ = enc_prev_valprev & 0xff;
						*cp = enc_prev_index;
						enc_prev_valprev = enc_valprev;
						enc_prev_index = enc_index;
						txseqno++;
						
						if (host_txseqno) host_txseqno++;

					}

					txseqno++;
	
					if (host_txseqno) host_txseqno++;
	
					filled = 1;
					fillindex = 0;
					filling_buffer ^= 1;
					last_drainindex = txdrainindex;
					timing_index = next_index;
					timing_time = next_time;
					apeak = (long)(amax - amin) / 2;
					for(i = NAPEAKS -1; i; i--) apeaks[i] = apeaks[i - 1];
					apeaks[0] = apeak;
				}
			}
		}
	}
}

/******************************************************************************
//									     //
//	These ISR's are not used					     //
//									     //
******************************************************************************/

void __attribute__((interrupt, auto_psv)) _DefaultInterrupt(void)
{
   Nop();
   Nop();
}

void __attribute__((interrupt, auto_psv)) _OscillatorFail(void)
{
   Nop();
   Nop();
}

void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
{
   Nop();
   Nop();
}

void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
{
   Nop();
   Nop();
}

void __attribute__((interrupt, auto_psv)) _MathError(void)
{
   Nop();
   Nop();
}
/*****************************************************************************/


int myfgets(char *buffer, unsigned int len);

#if defined(SMT_BOARD)

ROM WORD ledmask[] = {0x1000,0x800,0x400,0x2000};

void SetLED(BYTE led,BOOL val)
{
	LATB &= ~ledmask[led];
	
	if (!val) LATB |= ledmask[led];
}
	
void ToggleLED(BYTE led)
{
	LATB ^= ledmask[led];
}
	
void SetPTT(BOOL val)
{
	_LATB4 = val;	
}
	
void SetAudioSrc(void)
{
	BYTE myflags;

	if (!connected) 
	{
		myflags = 0;

		if (AppConfig.CORType || AppConfig.OffLineNoDeemp) myflags |= 1;

		if (AppConfig.Sawyer == 1) myflags |= 4;
	}
	else myflags = option_flags;

	if (myflags & 1)_LATB3 = 1;
	else _LATB3 = 0;

	if (!(myflags & 4)) _LATB2 = 1;
	else _LATB2 = 0;
}

#else

#define PROPER_SPICON1  (0x0003 | 0x0120)   /* 1:1 primary prescale, 8:1 secondary prescale, CKE=1, MASTER mode */
#define ClearSPIDoneFlag()
	
static inline __attribute__((__always_inline__)) void WaitForDataByte( void )
{
	while ((IOEXP_SPISTATbits.SPITBF == 1) || (IOEXP_SPISTATbits.SPIRBF == 0));
}
	
#define SPI_ON_BIT (IOEXP_SPISTATbits.SPIEN)
	
void IOExp_Write(BYTE reg,BYTE val)
{
	volatile BYTE vDummy;
	BYTE vSPIONSave;
	WORD SPICON1Save;
	
	// Save SPI state
	SPICON1Save = IOEXP_SPICON1;
	vSPIONSave = SPI_ON_BIT;
	
	// Configure SPI
	SPI_ON_BIT = 0;
	IOEXP_SPICON1 = PROPER_SPICON1;
	SPI_ON_BIT = 1;
	
	SPISel(SPICS_IOEXP);
	IOEXP_SSPBUF = 0x40;
	WaitForDataByte();
	vDummy = IOEXP_SSPBUF;
	IOEXP_SSPBUF = reg;
	WaitForDataByte();
	vDummy = IOEXP_SSPBUF;
	
	IOEXP_SSPBUF = val;
	WaitForDataByte();
	vDummy = IOEXP_SSPBUF;
	SPISel(SPICS_IDLE);
	    
	// Restore SPI State
	SPI_ON_BIT = 0;
	IOEXP_SPICON1 = SPICON1Save;
	SPI_ON_BIT = vSPIONSave;
	
	ClearSPIDoneFlag();
}
	
BYTE IOExp_Read(BYTE reg)
{	
	volatile BYTE vDummy,retv;
	BYTE vSPIONSave;
	WORD SPICON1Save;
	
	// Save SPI state
	SPICON1Save = IOEXP_SPICON1;
	vSPIONSave = SPI_ON_BIT;
	
	// Configure SPI
	SPI_ON_BIT = 0;
	IOEXP_SPICON1 = PROPER_SPICON1;
	SPI_ON_BIT = 1;
	
	SPISel(SPICS_IOEXP);
	IOEXP_SSPBUF = 0x41;
	WaitForDataByte();
	vDummy = IOEXP_SSPBUF;
	IOEXP_SSPBUF = reg;
	WaitForDataByte();
	vDummy = IOEXP_SSPBUF;
	IOEXP_SSPBUF = 0;
	WaitForDataByte();
	retv = IOEXP_SSPBUF;
	
	SPISel(SPICS_IDLE);
	// Restore SPI State
	SPI_ON_BIT = 0;
	IOEXP_SPICON1 = SPICON1Save;
	SPI_ON_BIT = vSPIONSave;
	
	ClearSPIDoneFlag();
	return retv;
}

// Initialize the IO Expander	
void IOExpInit(void)
{
	IOExp_Write(IOEXP_IOCON,0x20);
	IOExp_Write(IOEXP_IODIRA,0xD0);
	/* Make GPB6 an output only for non-SMT builds */
#if !defined(SMT_BOARD)
	/* Configure GPB directions: make GPB6 (bit 6) an output by clearing bit 6. */
	IODirB = 0x33; /* bit6=0 -> output */
#else
	/* Keep GPB6 as input on SMT builds */
	IODirB = 0x73; /* bit6=1 -> input */
#endif
	IOExp_Write(IOEXP_IODIRB,IODirB);
	IOExpOutA = 0xDF;
	IOExp_Write(IOEXP_OLATA,IOExpOutA);
	IOExpOutB = 0x53;
	IOExp_Write(IOEXP_OLATB,IOExpOutB);
}

void SetLED(BYTE led,BOOL val)
{
	BYTE mask,oldout;
	oldout = IOExpOutA;
	mask = 1 << led;
	IOExpOutA &= ~mask;
	
	if (!val) IOExpOutA |= mask;
	
	if (IOExpOutA != oldout) IOExp_Write(IOEXP_OLATA,IOExpOutA);
}
	
void ToggleLED(BYTE led)
{
	BYTE mask,oldout;
	oldout = IOExpOutA;
	mask = 1 << led;
	IOExpOutA ^= mask;
		
	if (IOExpOutA != oldout) IOExp_Write(IOEXP_OLATA,IOExpOutA);
}
	
void SetPTT(BOOL val)
{
	BYTE oldout;
	oldout = IOExpOutA;
	IOExpOutA &= ~0x20;

	if (val) IOExpOutA |= 0x20;
		
	if (IOExpOutA != oldout) IOExp_Write(IOEXP_OLATA,IOExpOutA);
}

#if !defined(SMT_BOARD)
static inline void SetConnStatus(BOOL val)
{
	BYTE oldout;
	BOOL actual_val;
	
	// Check AuxOutMode: 0=ConnStatus, 1=Force High, 2=Force Low
	if (AppConfig.AuxOutMode == 1)
		actual_val = 1;  // Force High
	else if (AppConfig.AuxOutMode == 2)
		actual_val = 0;  // Force Low
	else
		actual_val = val;  // Auto (use provided value)
	
	// Note: Offline delay is handled by the caller using delayed_connected variable
	// This function just sets the output based on the value provided
	
	oldout = IOExpOutB;
	IOExpOutB &= ~0x80;

	if (actual_val) IOExpOutB |= 0x80;
		
	if (IOExpOutB != oldout) IOExp_Write(IOEXP_OLATB,IOExpOutB);
}
#else
#define SetConnStatus(val)  do { } while(0)  // No-op for SMT boards
#endif

void SetAudioSrc(void)
{
	/* Determine the filtering for the receive audio.
	   ASEL1 = IOExpOutB mask Bit 4 = SPB2, setting to 1 is PL Filter IN, setting to 0 is PL Filter OUT
	   ASEL2 = IOExpOutB mask Bit 8 = SPB3, Setting to 1 is NO de-emphasis, setting to 0 is de-emphazised

	   myflags holds the bitmask
	   myflags 0 = de-emphazised,  PL filtered
	   myflags 1 = no de-emphasis, PL filtered
	   myflags 4 = de-emphasized,  no PL filter
	   myflags 5 = no de-emphasis, no PL filter

	   "Sawyer Mode" (Sawyer=1) forces the PL Filter OFF in Offline mode
	*/

	BYTE oldout,myflags;

	if (!connected) 
	{
		myflags = 0;

		if (AppConfig.CORType || AppConfig.OffLineNoDeemp) myflags |= 1;
			
		if (AppConfig.Sawyer == 1) myflags |= 4;
	}
	else myflags = option_flags;
	
	oldout = IOExpOutB;
	IOExpOutB &= ~0x0c;
	
	if (myflags & 1) IOExpOutB |= 8;
	
	if (!(myflags & 4)) IOExpOutB |= 4;

	if (IOExpOutB != oldout) IOExp_Write(IOEXP_OLATB,IOExpOutB);
}

#endif


void RTCM_Reset(void)
{
	SetPTT(0);
	while(!EmptyUART()) ClrWdt();
	Reset();
	while(1) DISABLE_INTERRUPTS();
}

BOOL HasCOR(void)
{
	if (AppConfig.CORType == 2) return(0);
	if (AppConfig.CORType == 1) return(1);
	return (cor);
}

BOOL HasCTCSS(void)
{
	if (!AppConfig.ExternalCTCSS) return (1);
	if (AppConfig.ExternalCTCSS == 3) return (0);
	if ((AppConfig.ExternalCTCSS == 1) && CTCSSIN) return (1);
	if ((AppConfig.ExternalCTCSS == 2) && (!CTCSSIN)) return(1);
	return (0);
}

void SetCTCSSTone(float freq, WORD gain)
{
	if ((freq <= 0.0) || (gain < 1))
	{
		tone_fac = 0;
		tone_v1 = 0;
		tone_v2 = 0;
		tone_v3 = 0;
		return;
	}

	tone_v1 = 0;
	// Last previous two samples
	tone_v2 = sin(-4.0 * M_PI * (freq / 8000.0)) * gain;
	tone_v3 = sin(-2.0 * M_PI * (freq / 8000.0)) * gain;
	// Frequency factor
	tone_fac = 2.0 * cos(2.0 * M_PI * (freq / 8000.0)) * 32768.0;
}


void SetTxTone(int freq)
{
	DISABLE_INTERRUPTS();
	if (freq == 0)
	{
		DAC1CONbits.DACFDIV = 74;	// Divide by 75 for 8K Samples/sec
		testp = 0;
		testidx = 0;
	}
	else
	{
		DAC1CONbits.DACFDIV = 36;	// Divide by 37 for approx 16216.216 Samples/sec
		testp = 0;
	}
	ENABLE_INTERRUPTS();
}

static void domorse(char *str)
{
	BYTE c;

	if (!cwptr)
	{
		while((c = *str++))
		{
			if (c < ' ') continue;
	
			if (c > 'Z') continue;
	
			c -= ' ';
	
			if (mbits[c].len)
			{
				cwlen = mbits[c].len * 2;
				cwdat = mbits[c].dat;
				if (cwdat & 1) cwtimer = AppConfig.CWSpeed * 3;
				else cwtimer = AppConfig.CWSpeed;
				cwdat = cwdat >> 1;
			}
			else
			{
				cwlen = 1;
				cwdat = 0;
				cwtimer = AppConfig.CWSpeed * 6;
			}
			break;
		}

		cwtimer1 = AppConfig.CWBeforeTime;
		cwptr = str;
		cwid_ptt_delay_timer = 24;  // 100ms delay @ ~4.125ms per tick
	}
}

BYTE GetBootCS(void)
{
	BYTE x = 0x69,i;

	for(i = 0; i < 4; i++) x += AppConfig.BootIPAddr.v[i];
	return(x);
}


/*
* Break up a delimited string into a table of substrings
*
* str - delimited string ( will be modified )
* strp- list of pointers to substrings (this is built by this function), NULL will be placed at end of list
* limit- maximum number of substrings to process
* delim- user specified delimeter
* quote- user specified quote for escaping a substring. Set to zero to escape nothing.
*
* Note: This modifies the string str, be suer to save an intact copy if you need it later.
*
* Returns number of substrings found.
*/

static int explode_string(char *str, char *strp[], int limit, char delim, char quote)
{
	int i,l,inquo;

        inquo = 0;
        i = 0;
        strp[i++] = str;

	if (!*str)
	{
		strp[0] = 0;
		return(0);
	}

	for(l = 0; *str && (l < limit) ; str++)
	{
		if (quote)
		{
			if (*str == quote)
			{
				if (inquo)
				{
					*str = 0;
					inquo = 0;
				}
				else
				{
					strp[i - 1] = str + 1;
					inquo = 1;
				}
			}
		}

		if ((*str == delim) && (!inquo))
		{
			*str = 0;
			l++;
			strp[i++] = str + 1;
		}
	}

	strp[i] = 0;
	return(i);
}

#define	memclr(x,y) 	memset(x,0,y)
#define ARPIsTxReady()	MACIsTxReady() 

WORD htons(WORD x)
{
	WORD y;
	BYTE *p = (BYTE *)&x;
	BYTE *q = (BYTE *)&y;

	q[0] = p[1];
	q[1] = p[0];
	return(y);
}

WORD ntohs(WORD x)
{
	WORD y;
	BYTE *p = (BYTE *)&x;
	BYTE *q = (BYTE *)&y;

	q[0] = p[1];
	q[1] = p[0];
	return(y);
}

DWORD htonl(DWORD x)
{
	DWORD y;
	BYTE *p = (BYTE *)&x;
	BYTE *q = (BYTE *)&y;

	q[0] = p[3];
	q[1] = p[2];
	q[2] = p[1];
	q[3] = p[0];
	return(y);
}

DWORD ntohl(DWORD x)
{
	DWORD y;
	BYTE *p = (BYTE *)&x;
	BYTE *q = (BYTE *)&y;

	q[0] = p[3];
	q[1] = p[2];
	q[2] = p[1];
	q[3] = p[0];
	return(y);
}

// Read an NMEA string from the GPS on UART2
BOOL getGPSStr(void)
{
	BYTE	c;

	while (DataRdyUART2())
	{
		c = ReadUART2();
	
		if (c == '\n')
		{
			gps_buf[gps_bufindex]= 0;
			gps_bufindex = 0;
			return 1; 
		}
	
		if (c < ' ') continue;
	
		gps_buf[gps_bufindex++] = c;
	
		if (gps_bufindex >= (sizeof(gps_buf) - 1))
		{
			gps_buf[gps_bufindex]= 0;
			gps_bufindex = 0;
			return 1; 
		}
	}
	return 0;
}

// Read a TSIP GPS packet on UART2
BOOL getTSIPPacket(void)
{
	BYTE     c;

	if (!DataRdyUART2()) return 0;
		
	c = ReadUART2();
        
	if (gps_bufindex == 0)
        {
                TSIPwasdle = 0;
        
	        if (c == 16)
                {
                        TSIPwasdle = 1;
                        gps_bufindex++;
                }
                return 0;
        }

        if (gps_bufindex > sizeof(gps_buf))
        {
                gps_bufindex = 0;
                return 0;
        }

        if ((c == 16) && (!TSIPwasdle))
        {
                TSIPwasdle = 1;
                return 0;
        }
        else if (TSIPwasdle && (c == 3))
        {
                gps_bufindex = 0;
                return 1;
        }

        TSIPwasdle = 0;
        gps_buf[gps_bufindex - 1] = c;
	gps_bufindex++;
        return 0;
}

static DWORD twoascii(char *s)
{
	DWORD rv;

	rv = s[1] - '0';
	rv += 10 * (s[0] - '0');
	return(rv);
}

/*****************************************************************************/
//									     //
//		Logtime Subroutine					     //
//									     //
/*****************************************************************************/
#define	logtime() logtime_p(&system_time)

static char *logtime_p(VTIME *p)
{
	time_t	t;
	static char str[50];
	static ROM char notime[] = "<System Time Not Set>",
	/* Use ISO-like ordering: YYYY/MM/DD for compact, sortable timestamps */
	logtemplate[] = "%Y/%m/%d %H:%M:%S";

	t = p->vtime_sec;
	
	if (t == 0) return((char *)notime);
	
	strftime(str,sizeof(str) - 1,(char *)logtemplate,gmtime(&t));
	sprintf(str + strlen(str),".%03lu",p->vtime_nsec / 1000000L);
	return(str);
}

// UTC time formatter helper - formats time in YYYY/MM/DD HH:MM:SS.mmm format
// This is used for log prefixes, menus, status displays, etc.
static char *format_utc_time(VTIME *p)
{
	time_t	t;
	static char str[50];
	static ROM char notime[] = "<System Time Not Set>";
	static ROM char utc_template[] = "%Y/%m/%d %H:%M:%S";

	t = p->vtime_sec;
	
	if (t == 0) return((char *)notime);
	
	strftime(str, sizeof(str) - 1, (char *)utc_template, gmtime(&t));
	sprintf(str + strlen(str), ".%03lu", p->vtime_nsec / 1000000L);
	return(str);
}

// Convenience macro to get current UTC time string
#define get_utc_time() format_utc_time(&system_time)

// Lightweight timestamp prefixer for consistent console logs with minimal ROM cost
static ROM char log_prefix_fmt[] = "[%s] ";
static inline void log_prefix(void)
{
	// Prints: "[YYYY/MM/DD HH:MM:SS.mmm] " using format_utc_time()
	printf(log_prefix_fmt, get_utc_time());
}


/*****************************************************************************/
//									     //
//		Calculate epoch Subroutine (mktime() replacement)	     //
//									     //
/*****************************************************************************/
// 01/06/21 WA1JHK patch to replace broken mktime() in MPLAB C30
static DWORD getSecondsSinceEpoch (struct tm *tm)
{
    #define SECONDS_EPOCH_TO_1121 1609459200  // seconds 1/1/1970 until 1/1/2021 0:0:0
    #define SECONDS_PER_DAY 86400             // 60 * 60 * 24
    #define SECONDS_PER_YEAR 31536000         // SECONDS_PER_DAY * 365

    DWORD   total_seconds;
    
    // days before current month in current year
    static ROM int normal_year[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    

    // SECONDS_EPOCH_TO_1121 is seconds from 1/1/70 0:0:0 up to 1/1/21 0:0:0
    total_seconds = SECONDS_EPOCH_TO_1121;
    // seconds elapsed current day since midnight
    total_seconds = total_seconds + ((DWORD)tm->tm_sec + ((DWORD)tm->tm_min * 60) + ((DWORD)tm->tm_hour * 3600));
    // seconds elapsed since 1st of month up to current day
    total_seconds = total_seconds + (((DWORD)tm->tm_mday - 1) * SECONDS_PER_DAY);
    // seconds elapsed since 1st of year up to current month
    total_seconds = total_seconds + (normal_year[tm->tm_mon - 1] * SECONDS_PER_DAY);
    // seconds elapsed since 1st of year up to current month
    total_seconds = total_seconds + ((tm->tm_year - 21) * SECONDS_PER_YEAR);
    // seconds for leap day added for March thru December in leap year
    if (((tm->tm_year % 4) == 0) & (tm->tm_mon > 2))
    {
        total_seconds = total_seconds + SECONDS_PER_DAY;
    }
    // seconds for extra leap days for all past years
    total_seconds = total_seconds + (((tm->tm_year - 21) / 4) * SECONDS_PER_DAY);

    return  (total_seconds);
}


/*****************************************************************************/
//									     //
//		Process GPS Subroutine					     //
//									     //
/*****************************************************************************/
void process_gps(void)
{
	int n;
	char *strs[30];
	static ROM char 	gpgga[] = "$GPGGA",
				gpgsv[] = "$GPGSV", 
				gprmc[] = "$GPRMC";
	static BYTE last_gps_state_logged = GPS_STATE_IDLE;
	static char last_gprmc_status = 0;

	// Please see doubleify.c for explanation of this poo-poo
	extern float doubleify(BYTE *p);

	if (gps_state == GPS_STATE_IDLE) gps_time = 0;

	if ((gpssync || (!USE_PPS)) && (gps_state == GPS_STATE_VALID))
	{
		gps_state = GPS_STATE_SYNCED;
		LOG_GPS("%s", gpsmsg3);
		last_gps_state_logged = GPS_STATE_SYNCED;
		main_processing_loop();
	}

	if ((!gpssync) && USE_PPS && (gps_state == GPS_STATE_SYNCED))
	{
		gps_state = GPS_STATE_VALID;
		LOG_GPS("%s", gpsmsg5);
		last_gps_state_logged = GPS_STATE_VALID;

		if (USE_PPS)
		{
			connected = 0;
			txseqno = 0;
			txseqno_ptt = 0;
			resp_digest = 0;
			digest = 0;
			their_challenge[0] = 0;
			lastrxtimer = 0;
			SetAudioSrc();
		}
	}

	if (AppConfig.GPSProto == GPS_NMEA)
	{
		if (!getGPSStr()) return;

		if ((AppConfig.DebugLevel & 32) && strstr((char *)gps_buf,gprmc))
		{
			LOG_GPS_DEBUG("%s\n",gps_buf);

			if ((ppsx) && (AppConfig.PPSPolarity <= 1)) 
			{ 
				LOG_GPS_DEBUG("PPS Configured but no pulse found, check polarity?\n"); 
			}
		}

		n = explode_string((char *)gps_buf,strs,30,',','\"');
	
		if (n < 1) return;
	
		if (!strcmp(strs[0],gpgsv))
		{
			if (n >= 4) gps_nsat = atoi(strs[3]);
			return;
		}
	
		if (!strcmp(strs[0],gprmc))
		{
			struct tm tm;
	
			if (n < 10) return;
			// Example NMEA GPS String
			// $GPRMC,194013.00,A,4032.94888,N,10511.83890,W,0.005,,020121,,,D*62
			//        hhmmss                                        ddmmyy
			// Use tm to pass binary time to getSecondsSinceEpoch

		// Require GPRMC status 'A' (valid) before accepting the time
		char current_status = (strs[2] && strs[2][0]) ? strs[2][0] : 'V';
		
		if (current_status != 'A')
		{
			// Only log on status change to avoid spam
			if (last_gprmc_status != current_status)
			{
				LOG_GPS("GPRMC '%c' void\n", current_status);
				last_gprmc_status = current_status;
			}
			return;
		}
		
		// Status is 'A' (valid) - log transition if changed
		if (last_gprmc_status != 'A')
		{
			LOG_GPS("GPRMC 'A' valid\n");
			last_gprmc_status = 'A';
		}			memset(&tm,0,sizeof(tm));
			tm.tm_sec = twoascii(strs[1] + 4);
			tm.tm_min = twoascii(strs[1] + 2);
			tm.tm_hour = twoascii(strs[1]);
			tm.tm_mday = twoascii(strs[9]);

			tm.tm_mon = twoascii(strs[9] + 2); // no longer need to -1, not using mktime()

			tm.tm_year = twoascii(strs[9] + 4); // don't need to be relative to 1900, not using mktime()
			if (AppConfig.DebugLevel & 64)
				gps_time = (DWORD) getSecondsSinceEpoch(&tm) + 1 + (DWORD) AppConfig.GPSOffset; // Fix for GPS one second off
			else
				gps_time = (DWORD) getSecondsSinceEpoch(&tm) + (DWORD) AppConfig.GPSOffset;

	LOG_GPS_DEBUG("mon: %d, gps_time: %ld, ctime: %s\n",tm.tm_mon,gps_time,ctime((time_t *)&gps_time));

	if (!USE_PPS) system_time.vtime_sec = timing_time = real_time = gps_time + 1;
		return;
	}		if (n < 7) return;
	
		if (strcmp(strs[0],gpgga)) return;
	
		gpswarn = 0;
		gpstimer = 0;
	
		if (gps_state == GPS_STATE_IDLE)
		{
			gps_state = GPS_STATE_RECEIVED;
			gpstimer = 0;
			gpswarn = 0;
			LOG_GPS("%s", gpsmsg1);
			last_gps_state_logged = GPS_STATE_RECEIVED;
		}
		n = atoi(strs[6]);

		LOG_GPS_DEBUG("GPGGA fix-quality=%d, sats=%d\n", n, gps_nsat);

		if (!gps_time)
		{
			LOG_GPS_DEBUG("GPGGA fix=%d, awaiting time parse\n", n);
		}
		if ((n < 1) || (n > 2)) 
		{
			if (gps_state == GPS_STATE_RECEIVED) return;

			gps_state = GPS_STATE_IDLE;
			LOG_GPS("%s", gpsmsg6);
			last_gps_state_logged = GPS_STATE_IDLE;

			if (USE_PPS)
			{
				connected = 0;
				txseqno = 0;
				txseqno_ptt = 0;
				resp_digest = 0;
				digest = 0;
				their_challenge[0] = 0;
				gpssync = 0;
				gotpps = 0;
				lastrxtimer = 0;
				SetAudioSrc();
			}
	}
		if ((gps_state == GPS_STATE_RECEIVED) && (gps_nsat > 0) && gps_time)
		{
			gps_state = GPS_STATE_VALID;
			LOG_GPS("%s%d\n", gpsmsg2, gps_nsat);
			last_gps_state_logged = GPS_STATE_VALID;
		}
	
		memclr(&gps_packet,sizeof(gps_packet));
		strncpy(gps_packet.lat,strs[2],7);
		gps_packet.lat[7] = *strs[3];
		strncpy(gps_packet.lon,strs[4],8);
		gps_packet.lon[8] = *strs[5];
		strncpy(gps_packet.elev,strs[9],6);
	}
	else /* is a Trimble TSIP Device */
	{
/* We are looking for two types of packets from TSIP devices:
   The 0x8f-ab Primary Timing Packet
   The 0x8f-ac Supplementary Timing Packet

   Both packets are read in to the gps_buf array. NOTE, our array count is off by one, if you 
   are looking at the TSIP reference, since gps_buf[0] contains the Header Byte, instead of the 
   Subcode byte.
*/
		if (!getTSIPPacket()) return;

		if (gps_buf[0] != 0x8f) return; // "Superpacket" Header

		if (gps_buf[1] == 0xab) // AB is the Primary Timing Packet
		{
			struct tm tm;
			WORD w;
	
			memset(&tm,0,sizeof(tm));
			tm.tm_sec = gps_buf[11]; // seconds 0-59
			tm.tm_min = gps_buf[12]; // minutes 0-59
			tm.tm_hour = gps_buf[13]; // hours 0-23
			tm.tm_mday = gps_buf[14]; // day of month 1-31

			/* gps_buf[15] is Month of Year, 1-12 */
			tm.tm_mon = gps_buf[15]; // no longer need to -1 as we are not using mktime() 

			w = gps_buf[17] | ((WORD)gps_buf[16] << 8); // 4-digit year (two bytes)
			tm.tm_year = w - 2000; // tm_year is now relative to years since 2000 (not using mktime())

			gpsweek = gps_buf[7] | ((WORD)gps_buf[6] << 8); // gps week number (two bytes)

			if (!AppConfig.GPSTbolt) // if this isn't a Tbolt device, don't fudge the time
			{
				gps_time = (DWORD) getSecondsSinceEpoch(&tm) + (DWORD) AppConfig.GPSOffset;
			}
			else
			{
			/* This is a Tbolt, so the firmware is broken. We need to figure out what week 
			it thinks it is, and correct it.*/
				if ((gpsweek >= 0) && (gpsweek <= 935)) // for weeks 0-935, add 1024 weeks
				{
					gps_time = (DWORD) getSecondsSinceEpoch(&tm) + (DWORD) ADD_1024_WEEKS + (DWORD) AppConfig.GPSOffset;
				}

				if ((gpsweek >= 936) && (gpsweek <= 1023)) // for weeks 936-1023, add 2048 weeks
				{
					gps_time = (DWORD) getSecondsSinceEpoch(&tm) + (2 * (DWORD) ADD_1024_WEEKS) + (DWORD) AppConfig.GPSOffset;
				}

				if (gpsweek >= 1024) // this isn't a Tbolt, so don't fudge the time
				{
					gps_time = (DWORD) getSecondsSinceEpoch(&tm) + (DWORD) AppConfig.GPSOffset;
				}
			}

			gpsleap = gps_buf[9] | ((WORD)gps_buf[8] << 8); // GPS-UTC offset (leap seconds)

			/* If the timing flags are all 0, then we are using GPS time, 
			   GPS PPS, time is set, we have UTC info (leap seconds), and we 
			   are getting time from the GPS. Therefore, we need to correct the time to UTC 
			   by subtracting the leap seconds, so that we can co-exist with other GPS that are 
			   reporting time in UTC. Otherwise, this device will be ignored by chan_voter, 
			   because the time will be offset (by leap seconds). */
			if (!gps_buf[10]) 
			{
				gps_time = gps_time - gpsleap;
			}
			
			LOG_GPS_DEBUG("gps_epoch_time: %ld, ctime: %s, gps_week: %d\n",gps_time,ctime((time_t *)&gps_time),gpsweek);
			
			if ((ppsx) && (AppConfig.PPSPolarity <= 1)) 
			{ 
				LOG_GPS_DEBUG("PPS Configured but no pulse found, check polarity?\n"); 
			}

			if (!USE_PPS) system_time.vtime_sec = timing_time = gps_time + 1;
		 	return;
		}

		if (gps_buf[1] == 0xac) // AC is the Supplemental Timing Packet
		{
			BOOL happy;
			int x,y;
			float f;

			happy = 1;

			if (gps_buf[13]) happy = 0; // GPS Decoding Status - 0=doing fixes

			if ((gps_buf[14] != 0) && (gps_buf[14] != 8)) happy = 0; // Discipline Activity, Phase Locked and Recovery Mode?

			if (gps_buf[9] || gps_buf[10]) happy = 0; // 0=No Critical Alarms

			if ((gps_buf[11] & 0x1f) | gps_buf[12]) happy = 0; // 0=No Minor Alarms
			/* Minor alarms are tricky! gps_buf[11] is Bits 8-12, gps_buf[12] is Bits 0-7
			ie gps_buf[12]=0x0a -> Antenna Open, Not Tracking Satellites
			   gps_buf[11]=0x08 -> Almanac not complete */

			LOG_GPS_DEBUG("TSIP: ok %d, 2,3,9 - 14: %02x %02x %02x %02x %02x %02x %02x %02x\n",
				happy,gps_buf[2],gps_buf[3],gps_buf[9],gps_buf[10],gps_buf[11],gps_buf[12],gps_buf[13],gps_buf[14]);

			gpswarn = 0;
			gpstimer = 0;

			if (gps_state == GPS_STATE_IDLE)
			{
				gps_state = GPS_STATE_RECEIVED;
				gpstimer = 0;
				gpswarn = 0;
				LOG_GPS("%s", gpsmsg1);
				last_gps_state_logged = GPS_STATE_RECEIVED;
			}

			if (!happy)
			{
				if (gps_state == GPS_STATE_RECEIVED) return;

				gps_state = GPS_STATE_IDLE;
				LOG_GPS("%s", gpsmsg6);
				last_gps_state_logged = GPS_STATE_IDLE;

				if (USE_PPS)
				{
					connected = 0;
					txseqno = 0;
					txseqno_ptt = 0;
					resp_digest = 0;
					digest = 0;
					their_challenge[0] = 0;
					lastrxtimer = 0;
					SetAudioSrc();
				}

				gpssync = 0;
				gotpps = 0;
			}

			gps_nsat = 3; // Hah, we're faking the number of received sats.

			if ((gps_state == GPS_STATE_RECEIVED) && (gps_nsat > 0) && gps_time)
			{
				gps_state = GPS_STATE_VALID;
				LOG_GPS("%s", gpsmsg9);
				last_gps_state_logged = GPS_STATE_VALID;
			}

			memclr(&gps_packet,sizeof(gps_packet));
			f = doubleify(gps_buf + 37) * TSIP_FACTOR;
			x = (int) f;
			f -= (float) x;

			if (f < 0.0) f = -f;

			f *= 60.0;
			y = (int) f;
			f -= (float) y;

			if (f < 0.0) f = -f;

			if (x < 0)
				sprintf(gps_packet.lat,"%02d%02d.%02dS",-x,y,(int)((f * 100.0) + 0.5));
			else
				sprintf(gps_packet.lat,"%02d%02d.%02dN",x,y,(int)((f * 100.0) + 0.5));

			f = doubleify(gps_buf + 45) * TSIP_FACTOR;
			x = (int) f;
			f -= (float) x;

			if (f < 0.0) f = -f;

			f *= 60.0;
			y = (int) f;
			f -= (float) y;

			if (f < 0.0) f = -f;

			if (x < 0)
				sprintf(gps_packet.lon,"%03d%02d.%02dW",-x,y,(int)((f * 100.0) + 0.5));
			else
				sprintf(gps_packet.lon,"%03d%02d.%02dE",x,y,(int)((f * 100.0) + 0.5));

			sprintf(gps_packet.elev,"%4.1f",(double)doubleify(gps_buf + 53));
			return;
		}
	}
	return;
}

/*****************************************************************************/
//									     //
//		ADPCM Decoder Subroutine				     //
//									     //
/*****************************************************************************/
void adpcm_decoder(BYTE *indata)
{
	BYTE *inp;		// Input buffer pointer 
	int sign;		// Current adpcm sign bit 
	BYTE delta;		// Current adpcm output value 
	int step;		// Stepsize 
	long valpred;	// Predicted value 
	int vpdiff;		// Current change to valpred 
	int index;		// Current step change index 
	BYTE inputbuffer;	// place to keep next 4-bit value 
	BOOL bufferstep;	// toggle between inputbuffer/input 
	WORD i;
	short sample, musign, exponent, mantissa;
	BYTE ulawbyte;

	inp = indata;

	valpred = dec_valprev;
	index = dec_index;
	step = stepsizeTable[index];
	bufferstep = 0;
	inputbuffer = 0;


	for ( i = 0; i < FRAME_SIZE * 2; i++) 
	{
		/* Step 1 - get the delta value */
		if ( bufferstep ) 
		{
			delta = inputbuffer & 0xf;
		} 
		else 
		{
			inputbuffer = *inp++;
			delta = (inputbuffer >> 4) & 0xf;
		}

		bufferstep = !bufferstep;
	
		/* Step 2 - Find new index value (for later) */
		index += indexTable[delta];
		
		if ( index < 0 ) index = 0;
		
		if ( index > 88 ) index = 88;
	
		/* Step 3 - Separate sign and magnitude */
		sign = delta & 8;
		delta = delta & 7;
	
		/* Step 4 - Compute difference and new predicted value */
		/*
		** Computes 'vpdiff = (delta+0.5)*step/4', but see comment
		** in adpcm_coder.
		*/
		vpdiff = step >> 3;
	
		if ( delta & 4 ) vpdiff += step;
	
		if ( delta & 2 ) vpdiff += step >> 1;
	
		if ( delta & 1 ) vpdiff += step >> 2;
	
		if ( sign )
			valpred -= vpdiff;
		else
			valpred += vpdiff;
	
		/* Step 5 - clamp output value */
		if ( valpred > 32767 )
			valpred = 32767;
		else if ( valpred < -32768 )
			valpred = -32768;
	
		/* Step 6 - Update step value */
		step = stepsizeTable[index];
	
		/* Step 7 - Output value */
//		vout = valpred + 32768;
			
		sample = valpred;
        
		/* Get the sample into sign-magnitude. */
		musign = (sample >> 8) & 0x80;	/* set aside the sign */
        
		if (musign != 0)
			sample = -sample;	/* get magnitude */
        
		if (sample > CLIP)
			sample = CLIP;		/* clip the magnitude */

		/* Convert from 16 bit linear to ulaw. */
		sample = sample + BIAS;
		exponent = exp_lut[(sample >> 7) & 0xFF];
		mantissa = (sample >> (exponent + 3)) & 0x0F;
		ulawbyte = ~(musign | (exponent << 4) | mantissa);

		dec_buffer[i] = ulawbyte;
    }

    dec_valprev = valpred;
    dec_index = index;
}

/*****************************************************************************/
//									     //
//		Process UDP Packet Subroutine				     //
//									     //
/*****************************************************************************/
void process_udp(UDP_SOCKET *udpSocketUser,NODE_INFO *udpServerNode)
{
	BYTE n,c,i,j,*cp;
#ifdef	DSPBEW
	short x;
	unsigned int *wp;
	BOOL qualnoise;
#endif

	WORD mytxindex;
	VTIME mysystem_time;
	long mytxseqno,myhost_txseqno;

	mytxindex = last_drainindex;
	mysystem_time = system_time;
	mytxseqno = txseqno;
	myhost_txseqno = host_txseqno;

	if (filled && (gpssync || (!USE_PPS)) && (!time_filled))
	{
		system_time.vtime_sec = timing_time;
		system_time.vtime_nsec = (timing_index / 160) * 20000000ul; 
		time_filled = 1;
	}

	if (filled && ((fillindex > MASTER_TIMING_DELAY) || (option_flags & OPTION_FLAG_MASTERTIMING)))
	{
	
#ifdef	DSPBEW
		for(i = 0; i < FFT_BLOCK_LENGTH; i++)
		{
			x = ulawtabletx[audio_buf[filling_buffer ^ 1][i]];

			if (AppConfig.BEWMode > 1)
			{
				if (x > 16383) x = 16383;
				if (x < -16383) x = -16383;
				sigCmpx[i].real = x;
			}
			else
			{
				sigCmpx[i].real = x / 2;
			}

			sigCmpx[i].imag = 0x0000;
		}
	
#ifndef FFTTWIDCOEFFS_IN_PROGMEM
		FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], &twiddleFactors[0], COEFFS_IN_DATA);
#else
		FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], (fractcomplex *) __builtin_psvoffset(&twiddleFactors[0]), (int) __builtin_psvpage(&twiddleFactors[0]));
#endif
	
		/* Store output samples in bit-reversed order of their addresses */
		BitReverseComplex (LOG2_BLOCK_LENGTH, &sigCmpx[0]);
	 
		/* Compute the square magnitude of the complex FFT output array so we have a Real output vetor */
		SquareMagnitudeCplx(FFT_BLOCK_LENGTH, &sigCmpx[0], &sigCmpx[0].real);
	
		wp = (unsigned int *)&sigCmpx[0];
		fftresult = 0;

		// Get the total energy above CTCSS and below 2000 Hz
		for(i = 0; i < FFT_TOP_SAMPLE_BUCKET; i++)
		{
			if (i >= 2) fftresult += *wp;

			wp++;
		}

		qualnoise = ((fftresult <= FFT_MAX_RESULT));

		if (!AppConfig.BEWMode) qualnoise = 1;
	
		if (!qualnoise)
		{
			vnoise32 = lastvnoise32[2] = lastvnoise32[1] = lastvnoise32[0];
			rssiheld = calcrssi(vnoise32 >> 3);
		}
#endif
		if (gpssync || (!USE_PPS))
		{
			BOOL tosend = (connected && ((HasCOR() && HasCTCSS()) || (option_flags & OPTION_FLAG_SENDALWAYS)));

			if (AppConfig.CORType == 1) rssiheld = rssi = 255;
#ifdef	DSPBEW
			if (qualnoise || (!HasCOR())) rssiheld = rssi;
#else
			rssiheld = rssi;
#endif
			if ((((!connected) && (attempttimer >= ATTEMPT_TIME)) || tosend) && UDPIsPutReady(*udpSocketUser)) 
			{
				UDPSocketInfo[activeUDPSocket].remoteNode.MACAddr = udpServerNode->MACAddr;
				memclr(&audio_packet,sizeof(VOTER_PACKET_HEADER));
				audio_packet.vph.curtime.vtime_sec = htonl(system_time.vtime_sec);
				audio_packet.vph.curtime.vtime_nsec = 
					(!USE_PPS) ? htonl(mytxseqno) : htonl(system_time.vtime_nsec);
				strcpy((char *)audio_packet.vph.challenge,challenge);
				audio_packet.vph.digest = htonl(resp_digest);
				
				if (tosend) audio_packet.vph.payload_type = htons((option_flags & OPTION_FLAG_ADPCM) ? 3 : 1);
				
				i = 0;
				cp = (BYTE *) &audio_packet;
				for(i = 0; i < sizeof(VOTER_PACKET_HEADER); i++) UDPPut(*cp++);
				j = (option_flags & OPTION_FLAG_ADPCM) ? FRAME_SIZE + 3 : FRAME_SIZE;
				c = (option_flags & OPTION_FLAG_ADPCM) ? ADPCM_SILENCE : ULAW_SILENCE;

	            		if (tosend)
				{	
					if ((rssiheld > 0) && HasCOR() && HasCTCSS())
					{
						UDPPut(rssiheld);
						for(i = 0; i < j; i++) UDPPut(audio_buf[filling_buffer ^ 1][i]);
						elketimer = 0;
					}
					else
					{
						UDPPut(0);
						for(i = 0; i < j; i++) UDPPut(c);
					}
				}
				else if (!USE_PPS) UDPPut(OPTION_FLAG_MIX);

			//Send contents of transmit buffer, and free buffer
			UDPFlush();
			attempttimer = 0;
			}
		}
#ifdef	DSPBEW
		if (qualnoise)
		{
			lastvnoise32[0] = lastvnoise32[1];
			lastvnoise32[1] = lastvnoise32[2];
			lastvnoise32[2] = vnoise32;
		}
#endif
		filled = 0;
		time_filled = 0;
	}
	
	if (connected && (sendgps || (!USE_PPS)) && (gps_packet.lat[0] || (gpsforcetimer >= GPS_FORCE_TIME)))
	{
	        if (UDPIsPutReady(*udpSocketUser)) 
		{
			UDPSocketInfo[activeUDPSocket].remoteNode.MACAddr = udpServerNode->MACAddr;
			gps_packet.vph.curtime.vtime_sec = htonl(real_time);
			gps_packet.vph.curtime.vtime_nsec = htonl(0);
			strcpy((char *)gps_packet.vph.challenge,challenge);
			gps_packet.vph.digest = htonl(resp_digest);
			gps_packet.vph.payload_type = htons(2);
			
			// Send elements one at a time -- SWINE dsPIC33 archetecture!!!
			cp = (BYTE *) &gps_packet.vph;
			for(i = 0; i < sizeof(gps_packet.vph); i++) UDPPut(*cp++);
			if (gps_packet.lat[0])
			{
				cp = (BYTE *) gps_packet.lat;
				for(i = 0; i < sizeof(gps_packet.lat); i++) UDPPut(*cp++);
				cp = (BYTE *) gps_packet.lon;
				for(i = 0; i < sizeof(gps_packet.lon); i++) UDPPut(*cp++);
				cp = (BYTE *) gps_packet.elev;
				for(i = 0; i < sizeof(gps_packet.elev); i++) UDPPut(*cp++);
			}
		
		UDPFlush();
		sendgps = 0;
		gpsforcetimer = 0;
		}

		memclr(&gps_packet,sizeof(gps_packet));
	}

	if (((gpssync || (!USE_PPS)) && UDPIsGetReady(*udpSocketUser)) && DAC1CONbits.DACEN) 
	{
		n = 0;
		cp = (BYTE *) &audio_packet;
		
		while(UDPGet(&c))
		{
			if (n++ < sizeof(audio_packet)) *cp++ = c;	
		}
		
		if (n >= sizeof(VOTER_PACKET_HEADER)) 
		{
			/* if this is a new session */
			if (strcmp((char *)audio_packet.vph.challenge,their_challenge))
			{
				connected = 0;
				txseqno = 0;
				txseqno_ptt = 0;
				lastrxtimer = 0;
				resp_digest = crc32_bufs(audio_packet.vph.challenge,(BYTE *)AppConfig.Password);
				strcpy(their_challenge,(char *)audio_packet.vph.challenge);
				SetAudioSrc();
			}
			else 
			{
				if ((!digest) || (!audio_packet.vph.digest) || (digest != ntohl(audio_packet.vph.digest)) ||
					(ntohs(audio_packet.vph.payload_type) == 0))
				{
					mydigest = crc32_bufs((BYTE *)challenge,(BYTE *)AppConfig.HostPassword);
				
				if (mydigest == ntohl(audio_packet.vph.digest))
				{
					digest = mydigest;
			
				if (!connected) 
				{
					gpsforcetimer = 0;
					ptt_ignore_timer = 0;
				}
		
					connected = 1;
					ever_connected = 1;
					lastrxtimer = 0;						if (n > sizeof(VOTER_PACKET_HEADER)) option_flags = audio_packet.rssi;
						else option_flags = 0;
				
						if ((!USE_PPS) && (!(option_flags & OPTION_FLAG_MIX)))
						{
							if (n > sizeof(VOTER_PACKET_HEADER)) gotbadmix = 1; 

							connected = 0;
							txseqno = 0;
							txseqno_ptt = 0;
							digest = 0;
							lastrxtimer = 0;
							SetAudioSrc();
						}
						else SetAudioSrc();
					}
				else
				{
					connected = 0;
					txseqno = 0;
					txseqno_ptt = 0;
					digest = 0;
					lastrxtimer = 0;
					SetAudioSrc();
				}
			}
		else
		{
		BYTE wconnected;			wconnected = connected;

			if (!connected) 
			{
				gpsforcetimer = 0;
				ptt_ignore_timer = 0;
			}

				connected = 1;
				ever_connected = 1;
				// Don't call SetConnStatus here - let secondary_processing_loop handle it with delay logic
				lastrxtimer = 0;

				if (!wconnected) SetAudioSrc();					lastrxtimer = 0;

					if (ntohs(audio_packet.vph.payload_type) == 5) // PING
					{
						if (!pingtimer) // If okay to respond to a ping 
						{
					        	if (UDPIsPutReady(*udpSocketUser)) 
							{
								UDPSocketInfo[activeUDPSocket].remoteNode.MACAddr = udpServerNode->MACAddr;
								audio_packet.vph.curtime.vtime_sec = htonl(real_time);
								audio_packet.vph.curtime.vtime_nsec = htonl(0);
								strcpy((char *)audio_packet.vph.challenge,challenge);
								audio_packet.vph.digest = htonl(resp_digest);
								audio_packet.vph.payload_type = htons(5);

								// Send elements one at a time -- SWINE dsPIC33 archetecture!!!
								cp = (BYTE *) &audio_packet.vph;
								for(i = 0; i < n; i++) UDPPut(*cp++);

					            		UDPFlush();
								pingtimer = MIN_PING_TIME;
							}
						}
					}

					if ((ntohs(audio_packet.vph.payload_type) == 1) || (ntohs(audio_packet.vph.payload_type) == 3))
					{
						long index,ndiff;
						short mydiff;
						last_rxpacket_time.vtime_sec = ntohl(audio_packet.vph.curtime.vtime_sec);
						last_rxpacket_time.vtime_nsec = ntohl(audio_packet.vph.curtime.vtime_nsec);
						last_rxpacket_sys_time = system_time;
						last_rxpacket_inbounds = 0;

						if (!USE_PPS)
						{
							mytxseqno = txseqno;

							if (mytxseqno > (txseqno_ptt + 2)) 
								host_txseqno = 0;

							txseqno_ptt = mytxseqno;

							if (!host_txseqno) 
								myhost_txseqno = host_txseqno = ntohl(audio_packet.vph.curtime.vtime_nsec);

							index = (ntohl(audio_packet.vph.curtime.vtime_nsec) - myhost_txseqno);
							index *= FRAME_SIZE;

							if (AppConfig.TxBufferLength >= 1440) 
								index -= (FRAME_SIZE * 4);
							else if (AppConfig.TxBufferLength >= 1120)
								index -= (FRAME_SIZE * 3);
							else if (AppConfig.TxBufferLength >= 800)
								index -= (FRAME_SIZE * 2);
							else if (AppConfig.TxBufferLength >= 640)
								index -= FRAME_SIZE;
						}
						else
						{
							index = (ntohl(audio_packet.vph.curtime.vtime_sec) - system_time.vtime_sec) * 8000;
							ndiff = ntohl(audio_packet.vph.curtime.vtime_nsec) - system_time.vtime_nsec;
							index += (ndiff / 125000);
						}

						index += AppConfig.TxBufferLength - (FRAME_SIZE * 2);
						last_rxpacket_index = index;
			
			                        /* if in bounds */
                        			if ((index >= 0) && (index <= (AppConfig.TxBufferLength - (FRAME_SIZE * 2))))
                        			{	
							last_rxpacket_inbounds = 1;
						
							if (!USE_PPS)
							{
								if ((txseqno + (index / FRAME_SIZE)) > txseqno_ptt) 
									txseqno_ptt = (txseqno + (index / FRAME_SIZE));
							}
							else
							{
								if ((ntohl(audio_packet.vph.curtime.vtime_sec) > lastrxtime.vtime_sec) ||
									((ntohl(audio_packet.vph.curtime.vtime_sec) == lastrxtime.vtime_sec) &&
										(ntohl(audio_packet.vph.curtime.vtime_nsec) > lastrxtime.vtime_nsec)))
								{
									lastrxtime.vtime_sec = ntohl(audio_packet.vph.curtime.vtime_sec);
									lastrxtime.vtime_nsec = ntohl(audio_packet.vph.curtime.vtime_nsec);
								}
							}

                            				index += mytxindex;

					   		if (index > AppConfig.TxBufferLength) 
								index -= AppConfig.TxBufferLength;
							
							mydiff = AppConfig.TxBufferLength;

							if (ntohs(audio_packet.vph.payload_type) == 3)
							{
					  			mydiff -= ((short)index + (FRAME_SIZE * 2));
								dec_valprev = (audio_packet.audio[160] << 8) + audio_packet.audio[161];
								dec_index = audio_packet.audio[162];
								adpcm_decoder(audio_packet.audio);

								if (mydiff >= 0)
								{
									memcpy(txaudio + index,dec_buffer,FRAME_SIZE * 2);
								}
								else
								{
									memcpy(txaudio + index,dec_buffer,(FRAME_SIZE * 2) + mydiff);
									memcpy(txaudio,dec_buffer + ((FRAME_SIZE * 2) + mydiff),-mydiff);
								}
							}
							else
							{
					  			mydiff -= ((short)index + FRAME_SIZE);
	                            				
								if (mydiff >= 0)
								{	
									memcpy(txaudio + index,audio_packet.audio,FRAME_SIZE);
								}
								else
								{
									memcpy(txaudio + index,audio_packet.audio,FRAME_SIZE + mydiff);
									memcpy(txaudio,audio_packet.audio + (FRAME_SIZE + mydiff),-mydiff);
								}
							}
                        			}
						else /* not in bounds */
						{
							if (index > (AppConfig.TxBufferLength - (FRAME_SIZE * 2)))
								missed = index - (AppConfig.TxBufferLength - (FRAME_SIZE * 2));
							else
								missed = index;
							misstimer1 = PKT_MISS_TIME;

							if (!USE_PPS)
							{
								host_txseqno = 0;
							}
						}
					}
				}
			}
		}

		UDPDiscard();
	}
}

/*****************************************************************************/
//									     //
//		Main Processing Loop					     //
//									     //
/*****************************************************************************/
void main_processing_loop(void)
{

	// UDP State machine
	#define SM_UDP_SEND_ARP     0
	#define SM_UDP_WAIT_RESOLVE 1
	#define SM_UDP_RESOLVED     2

	static BYTE smUdp = SM_UDP_SEND_ARP;
	static DWORD  tsecWait = 0;           //General purpose wait timer
	IP_ADDR vaddr;

	if(!MACIsLinked())
	{
		connected = 0;
		txseqno = 0;
		txseqno_ptt = 0;
		resp_digest = 0;
		digest = 0;
		their_challenge[0] = 0;
		lastrxtimer = 0;
		SetAudioSrc();
		
		/* if MAC was linked and now isnt, advance linkstate */
		if (linkstate == 1) linkstate++;
	}
	else
	{
		/* if first time having link, advance linkstate */
		if (linkstate == 0) linkstate++;

		/* if had link, lost it, and now have it again, reset processor (silly TCP/IP stack!) */
		if (linkstate == 2) RTCM_Reset();
	}

	if ((!AppConfig.Flags.bIsDHCPEnabled) || (!AppConfig.Flags.bInConfigMode))
	{
		if (altdns)
		{
			if (AppConfig.AltVoterServerFQDN[0])
			{
				if (!dnstimer)
				{
					DNSEndUsage();
					dnstimer = 60;
					dnsdone = 0;
					if(DNSBeginUsage()) 
						DNSResolve((BYTE *)AppConfig.AltVoterServerFQDN,DNS_TYPE_A);
				}
				else
				{
					if ((!dnsdone) && (DNSIsResolved(&vaddr)))
					{
						if (DNSEndUsage())
						{
							if (memcmp(&MyAltVoterAddr,&vaddr,sizeof(IP_ADDR)))
								altdnsnotify = 1;
							MyAltVoterAddr = vaddr;
						} 
						else altdnsnotify = 2;
		
						dnsdone = 1;
						altdns = 0;
					}
				}
			}
			else
			{
				altdns = 0;
			}
		}
		else
		{
			if (AppConfig.VoterServerFQDN[0])
			{
				if (!dnstimer)
				{
					DNSEndUsage();
					dnstimer = 60;
					dnsdone = 0;
			
					if(DNSBeginUsage()) 
						DNSResolve((BYTE *)AppConfig.VoterServerFQDN,DNS_TYPE_A);
				}
				else
				{
	
					if ((!dnsdone) && (DNSIsResolved(&vaddr)))
					{
						if (DNSEndUsage())
						{
							if (memcmp(&MyVoterAddr,&vaddr,sizeof(IP_ADDR)))
								dnsnotify = 1;	
							MyVoterAddr = vaddr;
						} 
						else dnsnotify = 2;
			
						dnsdone = 1;
						altdns = 1;
					}
				}
			}
		}
		/* if was connected and not connected now */
		if (altconnected && (!connected))
		{
			althost ^= 1;

			if (althost && 
				((!AppConfig.AltVoterServerFQDN[0]) ||
					 (!MyAltVoterAddr.v[0]))) althost = 0;

			alttimer = 0;
			altchange = 1;
		}

		altconnected = connected;

		if (alttimer >= MAX_ALT_TIME)
		{
			althost ^= 1;
			if (althost && 
				((!AppConfig.AltVoterServerFQDN[0]) ||
					 (!MyAltVoterAddr.v[0]))) althost = 0;
			alttimer = 0;
			altchange = 1;
		}

		if (connected) alttimer = 0;

		if (althost) vaddr = MyAltVoterAddr;
		else vaddr = MyVoterAddr;

		if (memcmp(&LastVoterAddr,&vaddr,sizeof(IP_ADDR)))
		{
	
			if (udpSocketUser != INVALID_UDP_SOCKET) UDPClose(udpSocketUser);
						memclr(&udpServerNode, sizeof(udpServerNode));

			udpServerNode.IPAddr = vaddr;
			udpSocketUser = UDPOpen(AppConfig.MyPort, &udpServerNode, 
				(althost && AppConfig.AltVoterServerPort) ? AppConfig.AltVoterServerPort : AppConfig.VoterServerPort);
			smUdp = SM_UDP_SEND_ARP;
			CurVoterAddr = vaddr;
			altchange1 = 1;

			if (lastalthost == althost)
			{
				connected = 0;
				txseqno = 0;
				txseqno_ptt = 0;
				resp_digest = 0;
				digest = 0;
				their_challenge[0] = 0;
				lastrxtimer = 0;
				SetAudioSrc();
			}
		}

		LastVoterAddr = vaddr;
		lastalthost = althost;
	
		if (connected && (lastrxtimer > LASTRX_TIME))
		{
			hosttimedout = 1;
			connected = 0;
			txseqno = 0;
			txseqno_ptt = 0;
			resp_digest = 0;
			digest = 0;
			their_challenge[0] = 0;
			lastrxtimer = 0;
			SetAudioSrc();
		}

		if (udpSocketUser != INVALID_UDP_SOCKET) switch (smUdp) 
		{
			case SM_UDP_SEND_ARP:
            			if (ARPIsTxReady()) 
				{
					//Remember when we sent last request
					tsecWait = TickGet();
                
					//Send ARP request for given IP address
					ARPResolve(&udpServerNode.IPAddr);
                
					smUdp = SM_UDP_WAIT_RESOLVE;
				}
				break;

			case SM_UDP_WAIT_RESOLVE:
				//The IP address has been resolved, we now have the MAC address of the
				//node at 10.1.0.101
				if (ARPIsResolved(&udpServerNode.IPAddr, &udpServerNode.MACAddr)) 
				{
					smUdp = SM_UDP_RESOLVED;
				}
					//If not resolved after 2 seconds, send next request
				else 
				{
					if ((TickGet() - tsecWait) >= TICK_SECOND / 2ul) 
					{
						smUdp = SM_UDP_SEND_ARP;
					}
				}
				break;

			case SM_UDP_RESOLVED:
				process_udp(&udpSocketUser,&udpServerNode);
				break;
		}
	}
	// This task performs normal stack task including checking
	// for incoming packet, type of packet and calling
	// appropriate stack entity to process it.
	StackTask();
    
	// This tasks invokes each of the core stack application tasks
	StackApplications();

	if ((termbufidx > 0) && (termbuftimer > TELNET_TIME)) ProcessTelnetTimer();
}
/***************End of Main Processing Loop***********************************/

/*****************************************************************************/
//									     //
//		Secondary Processing Loop				     //
//									     //
/*****************************************************************************/
void secondary_processing_loop(void)
{

	static DWORD t = 0, t1 = 0, t2 = 0, tdisp = 0;
	/* Audio stats timing / accumulation for debug option 2 */
	static DWORD last_aud_stats = 0;
	static DWORD rssi_sum_sec = 0;
	static WORD  rssi_count_sec = 0;
	static BYTE last_gps_state_logged = GPS_STATE_IDLE;
	static BOOL last_gotpps_logged = 0;
	static BOOL last_cor_logged = 0;
	static BOOL last_ctcss_logged = 0;
	static BOOL last_ptt_logged = 0;
	static BOOL last_connected_logged = 0;

	/* Deviation measurement averaging buffers (10Hz sampling) */
	#define DEV_BUF_15S 150  // 15 seconds at 10Hz
	static float dev_samples[DEV_BUF_15S];
	static WORD dev_sample_idx = 0;
	static WORD dev_samples_count = 0;

	long meas;
	WORD mypeak;
	long x,y,z;
	static BYTE dispcnt = 0;

	BOOL isoffline;
	BOOL qualtx;
	WORD g1;

#ifdef	DSPBEW
	BYTE qualnoise;
	static BYTE qualcnt = 255;
#endif


	static WORD mynoise;

#if !defined(SMT_BOARD)
	inputs1 = IOExp_Read(IOEXP_GPIOA);
	inputs2 = IOExp_Read(IOEXP_GPIOB);
#endif

	if (gps_state != GPS_STATE_IDLE)
		{
			if ((!gpswarn) && (gpstimer > ((AppConfig.GPSProto == GPS_TSIP) ? GPS_TSIP_WARN_TIME : GPS_NMEA_WARN_TIME)))
			{
				gpswarn = 1;
				LOG_WARN("GPS %s", gpsmsg7);
		}
		if (gpstimer >((AppConfig.GPSProto == GPS_TSIP) ? GPS_TSIP_MAX_TIME : GPS_NMEA_MAX_TIME))
		{
			LOG_GPS("%s", gpsmsg6);
			gps_state = GPS_STATE_IDLE;
			last_gps_state_logged = GPS_STATE_IDLE;
			if (USE_PPS)
			{
				connected = 0;
				resp_digest = 0;
				digest = 0;
				their_challenge[0] = 0;
				lastrxtimer = 0;
				SetAudioSrc();
			}

			gpssync = 0;
			gotpps = 0;
		}
	}		if (gotpps && USE_PPS)
		{
			if ((!ppswarn) && (ppstimer > PPS_WARN_TIME))
			{
				ppswarn = 1;
				LOG_WARN("GPS %s", gpsmsg8);
	}
		if (ppstimer > PPS_MAX_TIME)
		{
			LOG_GPS("%s", gpsmsg6);
			gps_state = GPS_STATE_IDLE;
			last_gps_state_logged = GPS_STATE_IDLE;
			last_gotpps_logged = 0;
			connected = 0;
			resp_digest = 0;
			digest = 0;
			their_challenge[0] = 0;
			gpssync = 0;
			gotpps = 0;
			lastrxtimer = 0;
			SetAudioSrc();
		}
	}
	process_gps();		if (sqlcount >= 33)
		{
			BOOL qualcor;
			sqlcount = 0;
			if (AppConfig.Sqpot) // check to see if we are using software squelch pot (yes if true)
			{
				service_squelch(adcothers[ADCDIODE],0x3ff - AppConfig.Squelch,adcothers[ADCSQNOISE],!CAL,!WVF,(AppConfig.SqlNoiseGain) ? 1: 0);
			}
			else // we're using the hardware pot
			{
				service_squelch(adcothers[ADCDIODE],0x3ff - adcothers[ADCSQPOT],adcothers[ADCSQNOISE],!CAL,!WVF,(AppConfig.SqlNoiseGain) ? 1: 0);
			}
			sql2 ^= 1;

			qualcor = (HasCOR() && HasCTCSS());	
#ifdef	DSPBEW
			qualnoise = ((fftresult <= FFT_MAX_RESULT) || (!qualcor)); 

			if (!AppConfig.BEWMode) qualnoise = 1;

			if (!qualnoise) qualcnt = 0;

			if (qualcnt <= QUALCOUNT)
			{
				qualcnt++;
				qualnoise = 0;
			}
#endif
			if (sql2)
			{
				if (qualcor && (!wascor))
				{
					lastvnoise32[0] = lastvnoise32[1] = lastvnoise32[2] = vnoise32 = (DWORD)adcothers[ADCSQNOISE] << 3;
				}
				else
				{
#ifdef	DSPBEW
					if (qualnoise) 
#endif
						vnoise32 = ((vnoise32 * 3) + ((DWORD)adcothers[ADCSQNOISE] << 3)) >> 2;
				}

			if ((connected == 0 && (AppConfig.OfflineDelay == 0 || conn_status_offline_timer >= (AppConfig.OfflineDelay * 242))) && 
				(!qualcor) && wascor && (gpssync || (!USE_PPS) || (!SIMULCAST_ENABLE)))
			{
				if (AppConfig.FailMode == 2) needburp = 1;

				if ((AppConfig.FailMode == 3) && (!connfail)) needburp = 1;
			}				wascor = qualcor;
			}
#ifdef	DSPBEW
			if (qualnoise) mynoise = (WORD)lastvnoise32[1];
#else
			mynoise = vnoise32;
#endif
		
			rssi = calcrssi(mynoise >> 3);


			if ((rssi < 1) && (qualcor)) rssiheld = rssi = 1;

			if (!AppConfig.SqlNoiseGain) rssiheld = rssi = 0;

			/* Accumulate RSSI samples for 1s averaging (used by debug option 2) */
			rssi_sum_sec += rssiheld;
			rssi_count_sec++;

			// Log COR state changes
			{
				BOOL current_cor = HasCOR();
				BOOL current_ctcss = HasCTCSS();
				
				if (current_cor != last_cor_logged)
				{
					LOG_RX("COR %s, RSSI=%d\n", current_cor ? log_detected : log_lost, rssi);
					last_cor_logged = current_cor;
				}
				
				if (current_ctcss != last_ctcss_logged)
				{
					LOG_RX("CTCSS %s\n", current_ctcss ? log_detected : log_lost);
					last_ctcss_logged = current_ctcss;
				}
			}

			lastcor = HasCOR();

			if (write_eeprom_cali)
			{
				write_eeprom_cali = 0;
				AppConfig.SqlNoiseGain = noise_gain;
				if (!WVF) AppConfig.SqlDiode = caldiode;
				SaveAppConfig();
				LOG_SYS("Sql cal, gain=%d\n", noise_gain);

				if (!WVF)
				{
					LOG_SYS("Diode cal=0x%04X\n", caldiode);
				}
			}

	if (!CAL) SetLED(SQLED,sqled);

	z = 100000;
	x = system_time.vtime_sec - lastrxtime.vtime_sec;
	
	// Manage offline delay timer (works on both SMT and non-SMT boards)
	if (connected)
	{
		conn_status_offline_timer = 0;  // Reset timer when online
	}
	else if (AppConfig.OfflineDelay > 0 && conn_status_offline_timer < (AppConfig.OfflineDelay * 242))
	{
		conn_status_offline_timer++;  // Increment timer during offline delay period
	}
	
	// Update connection status output with delay applied
	// Only apply delay if we've been connected before (ever_connected) and are now disconnected
	// This prevents showing "connected" at startup before first connection
	if (ever_connected && AppConfig.OfflineDelay > 0 && !connected && conn_status_offline_timer < (AppConfig.OfflineDelay * 242))
	{
		SetConnStatus(1);  // Delay not yet expired, show as connected during delay period
	}
	else
	{
		SetConnStatus(connected);  // Show actual connection state (delay expired or not configured)
	}
	
	// Calculate delayed connected state (applies offline delay)
	// If delay is configured and timer hasn't expired yet, treat as still connected
	isoffline = ((!connected || (AppConfig.OfflineDelay > 0 && connected == 0 && 
		conn_status_offline_timer < (AppConfig.OfflineDelay * 242))) ? 0 : 1);
	isoffline = (!isoffline) && (AppConfig.FailMode == 3);

		if ((isoffline || DUPLEX3) && HasCOR() && HasCTCSS() && (gpssync || (!SIMULCAST_ENABLE) || (!USE_PPS)))
		{
			repeatit = 1;

			if (isoffline) hangtimer = AppConfig.HangTime + 1;
			else hangtimer = AppConfig.Duplex3;

		} 
		else repeatit = 0;

		// Decrement CWID PTT delay timer if active
		if (cwid_ptt_delay_timer > 0) cwid_ptt_delay_timer--;

		if (cwptr || cwtimer1 || hangtimer)
		{
			// Only assert PTT if delay timer has expired (allows GPB7 to stabilize first)
			if (cwid_ptt_delay_timer == 0)
			{
				if (!last_ptt_logged)
				{
					LOG_TX("PTT %s(CW)\n", log_asserted);
					last_ptt_logged = 1;
				}
				host_ptt = 0;
				ptt = 1;
				SetPTT(1);
			}
		}
		else
		{
			if (HasCOR() && HasCTCSS()) g1 = glasertimer = AppConfig.Glasers;
			else g1 = 0;

			qualtx = ((!AppConfig.Elkes) ||
						(AppConfig.Elkes == 0xffffffff) || (elketimer < AppConfig.Elkes));
			qualtx &= (!((AppConfig.Glasers && (AppConfig.Glasers != 0xffff)) && (glasertimer || g1)));

			if (connected)
			{
				if (!USE_PPS)
				{
					if (ptt && ((txseqno > (txseqno_ptt + 2)) || (!qualtx)))
					{
						if (last_ptt_logged)
						{
							LOG_TX("PTT %s\n", log_deasserted);
							last_ptt_logged = 0;
						}
						host_ptt = 0;
						ptt = 0;
						SetPTT(0);
					}
					else if ((!ptt) && (txseqno <= (txseqno_ptt + 2)) && qualtx && (ptt_ignore_timer >= PTT_IGNORE_TIME))
					{
						if (!last_ptt_logged)
						{
							LOG_TX("PTT %s buf=%dms\n", log_asserted, AppConfig.TxBufferLength >> 3);
							last_ptt_logged = 1;
						}
						host_ptt = 1;
						ptt = 1;
						SetPTT(1);
					}
				}
				else
				{
					if (lastrxtime.vtime_sec && (x < 100) && (z <= 100000))
					{
						y = system_time.vtime_nsec - lastrxtime.vtime_nsec;
						z = x * 1000;
						z += y / 1000000;
						z -= (AppConfig.TxBufferLength - 160) >> 3;
					}

					if (ptt && ((z > 60L) || (!qualtx)))
					{
						if (last_ptt_logged)
						{
							LOG_TX("PTT %s\n", log_deasserted);
							last_ptt_logged = 0;
						}
						host_ptt = 0;
						ptt = 0;
						SetPTT(0);
					}
					else if ((!ptt) && (z <= 60L) && qualtx && (ptt_ignore_timer >= PTT_IGNORE_TIME))
					{
						if (!last_ptt_logged)
						{
							LOG_TX("PTT %s buf=%dms tim=%ldms\n", log_asserted, AppConfig.TxBufferLength >> 3, z);
							last_ptt_logged = 1;
						}
						host_ptt = 1;
						ptt = 1;
						SetPTT(1);
					}
				}
			}
			else 
			{
				if (last_ptt_logged)
				{
					LOG_TX("PTT %s(disc)\n", log_deasserted);
					last_ptt_logged = 0;
				}
				host_ptt = 0;
				ptt = 0;
				SetPTT(0);
			}
		}

		if (LEVDISP)
		{
			if ((!misstimer1) || (!connected)) SetLED(CONNLED,connected);

			if ((gps_state == GPS_STATE_SYNCED) ||
				((gps_state == GPS_STATE_VALID) && (!USE_PPS)))
					SetLED(GPSLED,1);
			else if (gps_state != GPS_STATE_VALID)
				SetLED(GPSLED,0);
		}
		else
		{
			if (HasCOR() && HasCTCSS())
			{
				mypeak = apeak / (7200 / LEVDISP_FACTOR);

				if (mypeak < dispcnt) SetLED(CONNLED,1);
				else SetLED(CONNLED,0);

				if (mypeak > (dispcnt + LEVDISP_FACTOR)) SetLED(GPSLED,1);
				else SetLED(GPSLED,0);
			}
			else
			{
				SetLED(GPSLED,0);
				SetLED(CONNLED,0);
			}
			dispcnt++;

			if (dispcnt > LEVDISP_FACTOR) dispcnt = 0;
		}
	
		if (CAL)
		{	
			if (lastcor && HasCTCSS())
				SetLED(SQLED,1);
			else if ((!lastcor) || ((AppConfig.CORType != 0) && (!HasCTCSS())))
				SetLED(SQLED,0);
		}
	}

	// Blink LEDs as appropriate
	if(TickGet() - t1 >= TICK_SECOND / 6ul)
	{
		t1 = TickGet();
	}

	// Blink LEDs as appropriate
	if(TickGet() - t2 >= TICK_SECOND / 10ul)
	{
		t2 = TickGet();
		
		if (misstimer1) ToggleLED(CONNLED);
	}

	// Blink LEDs as appropriate
	if(TickGet() - t >= TICK_SECOND / 2ul)
	{
		if (dnstimer) dnstimer--;
		
		alttimer++;
		t = TickGet();

		ToggleLED(SYSLED);

		if (LEVDISP)
		{
			if ((gps_state == GPS_STATE_VALID) && USE_PPS) ToggleLED(GPSLED);
		}

		if (CAL && (AppConfig.CORType == 0) && (lastcor && (!HasCTCSS()))) ToggleLED(SQLED);
	}

	/* Audio statistics: every 1 second while TXing or RXing, print stats when debug option 2 is enabled */
	if ((AppConfig.DebugLevel & 2) && (TickGet() - last_aud_stats >= TICK_SECOND))
	{
		last_aud_stats = TickGet();

		/* TX: print only when PTT is asserted */
		if (ptt)
		{
			/* Compute queued frames/ms from circular buffer indices */
			unsigned int queued_frames = 0;
			if (fillindex >= txdrainindex) queued_frames = fillindex - txdrainindex;
			else queued_frames = AppConfig.TxBufferLength - (txdrainindex - fillindex);
			unsigned int queued_ms = queued_frames >> 3; /* buffer ms = TxBufferLength >> 3 */

			LOG_STAT("TX STAT: tx=%ld host=%ld q=%ums drain=%u miss=%ld opt=0x%02X\n",
				txseqno, host_txseqno, queued_ms, txdrainindex, missed, option_flags);
		}

		/* RX: Only print when actually receiving and CTCSS (if required) */
		if (HasCOR() && HasCTCSS())
		{
			unsigned int avg = 0;
			unsigned long ms_since_rx = 0;
			if (rssi_count_sec) avg = (unsigned int)(rssi_sum_sec / rssi_count_sec);

			/* Convert RSSI (0-255) to percent for easier reading */
			unsigned int rssi_pct = 0;
			if (avg) rssi_pct = (unsigned int)(((unsigned long)avg * 100UL + 127UL) / 255UL);

			if (last_rxpacket_sys_time.vtime_sec)
			{
				long mydiff = system_time.vtime_sec - last_rxpacket_sys_time.vtime_sec;
				long mydiff1 = system_time.vtime_nsec - last_rxpacket_sys_time.vtime_nsec;
				mydiff *= 1000;
				mydiff1 /= 1000000;
				mydiff += mydiff1;
				if (mydiff < 0) mydiff = 0;
				ms_since_rx = (unsigned long) mydiff;
			}

			LOG_STAT("RX STAT: rssi=%u%% samples=%u last_samplecnt=%u missed=%ld ms=%lu inx=%ld inb=%d\n",
				rssi_pct, rssi_count_sec, last_samplecnt, missed, ms_since_rx, last_rxpacket_index, last_rxpacket_inbounds);
		}

		/* no snapshot needed when TX is gated strictly by PTT */

		/* reset accumulators */
		rssi_sum_sec = 0;
		rssi_count_sec = 0;
	}

	/* Cold Power-Up Auto Reboot - trigger N minutes after cold boot */
	if (cold_power_reboot_active && uptimer >= cold_power_reboot_target)
	{
		LOG_SYS("Cold reboot\n");
		RTCM_Reset();
	}

	/* Auto Reboot Scheduler - fire at second :00 of target minute */
	{
		static BYTE last_reboot_min = 0xFF;  // Track last minute we fired to prevent re-trigger
		if ((AppConfig.RebootMode > 0) && (system_time.vtime_sec > 0))
		{
			struct tm *tm = gmtime((time_t *)&system_time.vtime_sec);
			
			/* Fire at :00 seconds of target hour:minute (and day if weekly) */
			if (tm->tm_sec == 0 && 
			    tm->tm_hour == AppConfig.RebootHour && 
			    tm->tm_min == AppConfig.RebootMinute &&
			    tm->tm_min != last_reboot_min &&
			    ((AppConfig.RebootMode == 1) || 
			     (AppConfig.RebootMode == 2 && tm->tm_wday == AppConfig.RebootDay)))
			{
				last_reboot_min = tm->tm_min;
				LOG_SYS("Sched reboot\n");
				RTCM_Reset();
			}
			/* Clear flag when we move to a different minute */
			if (tm->tm_min != last_reboot_min && last_reboot_min != 0xFF)
			{
				last_reboot_min = 0xFF;
			}
		}
	}

	if ((!indipsw) && (!indisplay) && (!leddiag)) tdisp = 0;

	// Rx Level Display handler
	if(indisplay && (TickGet() - tdisp >= TICK_SECOND / 10ul))
	{
       		tdisp = TickGet();

		if (HasCOR() && HasCTCSS())
			meas = apeak;
		else
			meas = 0;

		{
			// Deviation calculation calibrated at 3.0 KHz reference point:
			// At 3.0 KHz actual deviation, firmware measures 2.73 KHz raw
			// Calibration multiplier: CAL_FACTOR = 3.0 / 2.73 = 1.0989 adjusted to 1.1062
			// The bar display gives: raw_khz = (meas * 5.0) / 12231
			// Calibrated formula: actual_khz = raw_khz * CAL_FACTOR
			// Combined: actual_khz = (meas * CAL_FACTOR * 5.0) / 12231
			// Using CAL_FACTOR = 1.1062 → actual_khz = (meas * 5.531) / 12231 = meas / 2211.0

			#define CAL_FACTOR 1.1062f  // deviation calibration multiplier (adjust as needed)

			float deviation_khz = ((float)meas * CAL_FACTOR * 5.0f) / 12231.0f;

			float avg_15s = 0;
			WORD count_15s;
			WORD j, idx;
			
			// Store current sample in circular buffer
			dev_samples[dev_sample_idx] = deviation_khz;
			dev_sample_idx = (dev_sample_idx + 1) % DEV_BUF_15S;
			if (dev_samples_count < DEV_BUF_15S) dev_samples_count++;
			
			// Calculate 15s average (most recent 150 samples at 10Hz)
			count_15s = (dev_samples_count < 150) ? dev_samples_count : 150;
			for (j = 0; j < count_15s; j++)
			{
				idx = (dev_sample_idx + DEV_BUF_15S - 1 - j) % DEV_BUF_15S;
				avg_15s += dev_samples[idx];
			}
			if (count_15s > 0) avg_15s /= count_15s;

			// Display on single line
			printf("Instant: %.2f KHz | 15s Avg: %.2f KHz\r",
			       (double)deviation_khz, (double)avg_15s);
			fflush(stdout);
		}
		
		amin = amax = 0;
	}

	// Dip Switch diagnostic display handler
	if(indipsw && (TickGet() - tdisp >= TICK_SECOND / 10ul))
	{
		tdisp = TickGet();
		printf(" (%s) (%s) (%s) (%s)\r",(!JP8) ? "Down" : " Up ",(!JP9) ? "Down" : " Up ",
			(!JP10) ? "Down" : " Up ",(!JP11) ? "Down" : " Up ");
		fflush(stdout);
	}
 
	// LED Flash diagnostic handler
	if(leddiag && (TickGet() - tdisp >= TICK_SECOND * 2ul))
	{
		tdisp = TickGet();
		SetLED(SQLED,0);
		SetLED(GPSLED,0);
		SetLED(CONNLED,0);
		SetPTT(0);

		switch (leddiag)
		{
			case 1:
				SetLED(SQLED,1);
			    	leddiag = 2;
				break;

			case 2:
				SetLED(CONNLED,1);
				leddiag = 3;
				break;
	
			case 3:
				SetPTT(1);
				leddiag = 4;
				break;

			case 4:
				SetLED(GPSLED,1);
				leddiag = 1;
				break;
		}
   	}


	if (gotbadmix)
	{
		LOG_ERR("%s", badmix);
		gotbadmix = 0;
	}

	if (hosttimedout)
	{
		LOG_ERR("%s", hosttmomsg);
		hosttimedout = 0;
	}

	if (dnsnotify == 1)
	{
		LOG_NET("DNS %s->", AppConfig.VoterServerFQDN);
		printf(fmt_ip_newline,MyVoterAddr.v[0],MyVoterAddr.v[1],MyVoterAddr.v[2],MyVoterAddr.v[3]);
	}
	else if (dnsnotify == 2) 
	{
		LOG_WARN("DNS fail %s\n", AppConfig.VoterServerFQDN);
	}

	dnsnotify = 0;

	if (altdnsnotify == 1)
	{
		LOG_NET("DNS %s(Alt)->", AppConfig.AltVoterServerFQDN);
		printf(fmt_ip_newline,MyAltVoterAddr.v[0],MyAltVoterAddr.v[1],MyAltVoterAddr.v[2],MyAltVoterAddr.v[3]);
	}
	else if (altdnsnotify == 2) 
	{
		LOG_WARN("DNS fail %s(Alt)\n", AppConfig.AltVoterServerFQDN);
	}

	altdnsnotify = 0;
	if (missed && (!misstimer))
	{
		LOG_PKT("OOB ofs=%ld samp (%ldms)\n", -missed, (-missed * 125) / 1000);
		misstimer = MISS_REPORT_TIME;
		missed = 0;
	}

	if ((!connected) && connrep)
		{
			LOG_NET("Disc(%s) %d.%d.%d.%d:%d\n", (althost) ? "Alt" : "Pri",
				CurVoterAddr.v[0],CurVoterAddr.v[1],CurVoterAddr.v[2],CurVoterAddr.v[3],
				AppConfig.VoterServerPort);
			connrep = 0;
			last_connected_logged = 0;
		}
		else if (connected && (!connrep))
		{
			LOG_NET("Conn(%s) %d.%d.%d.%d:%d\n", (althost) ? "Alt" : "Pri",
				CurVoterAddr.v[0],CurVoterAddr.v[1],CurVoterAddr.v[2],CurVoterAddr.v[3],
				AppConfig.VoterServerPort);
			connrep = 1;
			last_connected_logged = 1;
		}

		if (connected)
		{
			failtimer = 0;
			hangtimer = 0;
		}

		// Use delayed connection state for offline CW ID
		BOOL delayed_connected = (connected || (AppConfig.OfflineDelay > 0 && conn_status_offline_timer < (AppConfig.OfflineDelay * 242)));
		
		if (delayed_connected && (!connfail)) connfail = 1;
		else if (ever_connected && AppConfig.FailMode && (!cwptr) && (!cwtimer1) && (gpssync || (!SIMULCAST_ENABLE) || (!USE_PPS)))
		{
			if (delayed_connected)
			{
				if ((connfail == 2) && AppConfig.UnFailString[0])
				{
					domorse((char *)AppConfig.UnFailString);
					connfail = 1;
				}
			}
			else
			{
				if ((connfail == 1) && AppConfig.FailString[0])
				{
					domorse((char *)AppConfig.FailString);
					connfail = 2;
					failtimer = 0;
				}
			}

			if ((connfail == 2) && AppConfig.FailTime && 
				(failtimer >= AppConfig.FailTime) && AppConfig.FailString[0])
			{
				domorse((char *)AppConfig.FailString);
				failtimer = 0;
			}
		}

		if (delayed_connected) needburp = 0;

		if (ever_connected && needburp && (!cwptr) && (!cwtimer1) && AppConfig.FailString[0] && (gpssync || (!SIMULCAST_ENABLE) || (!USE_PPS)))
		{
			needburp = 0;
			if (!connfail) connfail = 2;
			domorse((char *)AppConfig.FailString);
			failtimer = 0;
		}

	// If the local IP address has changed (ex: due to DHCP lease change)
	// write the new IP address to the LCD display, UART, and Announce 
	// service
	if(dwLastIP != AppConfig.MyIPAddr.Val)
	{
		dwLastIP = AppConfig.MyIPAddr.Val;
		LOG_NET("%s\n", AppConfig.Flags.bIsDHCPReallyEnabled ? "DHCP" : "Static");
		LOG_NET("IP:");
		printf(fmt_ip_newline,AppConfig.MyIPAddr.v[0],AppConfig.MyIPAddr.v[1],AppConfig.MyIPAddr.v[2],AppConfig.MyIPAddr.v[3]);
		LOG_NET("Mask:");
		printf(fmt_ip_newline,AppConfig.MyMask.v[0],AppConfig.MyMask.v[1],AppConfig.MyMask.v[2],AppConfig.MyMask.v[3]);
		LOG_NET("GW:");
		printf(fmt_ip_newline,AppConfig.MyGateway.v[0],AppConfig.MyGateway.v[1],AppConfig.MyGateway.v[2],AppConfig.MyGateway.v[3]);

		#if defined(STACK_USE_ANNOUNCE)
			AnnounceIP();
		#endif
	}

	altchange = 0;
	altchange1 = 0;
}
/***************End of Secondary Processing Loop*******************************/


int write(int handle, void *buffer, unsigned int len)
{
	int i;
	BYTE *cp;
	i = len;

	if ((handle >= 0) && (handle <= 2))
		{
			cp = (BYTE *)buffer;
			while(i--)
			{
				while(BusyUART()) main_processing_loop();
			
				if (*cp == '\n')
				{
					WriteUART('\r');
			
					if (netisup) while(!PutTelnetConsole('\r')) main_processing_loop();
					while(BusyUART())main_processing_loop();
				}

				WriteUART(*cp);

				if (netisup) while (!PutTelnetConsole(*cp)) main_processing_loop();
				cp++;
			}
		}
		else
		{
			errno = EBADF;
			return(-1);
		}

	return(len);
}

int read(int handle,void *buffer, unsigned int len)
{
	return 0;
}

int myfgets(char *dest, unsigned int len)
{
	BYTE c;
	int count,x;
	ClrWdt();
	fflush(stdout);
	inread = 1;
	aborted = 0;
	count = 0;
	while(count < len)
	{
		dest[count] = 0;
		for(;;)
		{
			ClrWdt();
			if ((!netisup) && ((!AppConfig.Flags.bIsDHCPEnabled) || (!AppConfig.Flags.bInConfigMode)))
				netisup = 1;

			if (DataRdyUART())
			{
				c = ReadUART() & 0x7f;
				break;
			}

			if (netisup)
			{
				c = GetTelnetConsole();

				if (c == 4) 
				{
					CloseTelnetConsole();
					continue;
				}

				if (c) break;
			}

			main_processing_loop();
			secondary_processing_loop();
		}

		if (c == 127) continue;

		if (c == 3) 
		{
			while(BusyUART())  main_processing_loop();
			WriteUART('^');

			if (telnet_echo && netisup) while(!PutTelnetConsole('^'))  main_processing_loop();

			while(BusyUART())  main_processing_loop();
			WriteUART('C');

			if (telnet_echo && netisup) while(!PutTelnetConsole('C'))  main_processing_loop();

			aborted = 1;
			dest[0] = '\n';
			dest[1] = 0;
			count = 1;
			break;
		}

		if ((c == 8) || (c == 21))
		{
			if (!count) continue;

			if (c == 21) x = count;
			else x = 1;

			while(x--)
			{
				count--;
				dest[count] = 0;

				while(BusyUART())  main_processing_loop();
				WriteUART(8);

				if (netisup) while(!PutTelnetConsole(8))  main_processing_loop();

				while(BusyUART()) main_processing_loop();
				WriteUART(' ');

				if (netisup) while(!PutTelnetConsole(' '))  main_processing_loop();

				while(BusyUART()) main_processing_loop();
				WriteUART(8);

				if (netisup) while(!PutTelnetConsole(8))  main_processing_loop();
			}
			continue;
		}

		if (c == 4) 
		{
			if (netisup) CloseTelnetConsole();
			continue;
		}

		if ((c != '\r') && (c < ' ')) continue;

		if (c > 126) continue;

		if (c == '\r') c = '\n';

		dest[count++] = c;
		dest[count] = 0;

		if (c == '\n') break;

		while(BusyUART())  main_processing_loop();
		WriteUART(c);

		if (telnet_echo && netisup) while(!PutTelnetConsole(c))  main_processing_loop();

	}

	while(BusyUART())  main_processing_loop();
	WriteUART('\r');

	if (netisup) while(!PutTelnetConsole('\r'))  main_processing_loop();

	while(BusyUART())  main_processing_loop();
	WriteUART('\n');

	if (netisup) while(!PutTelnetConsole('\n'))  main_processing_loop();

	inread = 0;
	return(count);
}

/*****************************************************************************/
//									     //
//		DynDNS Subroutine					     //
//									     //
/*****************************************************************************/
static void SetDynDNS(void)
{
	static ROM BYTE checkip[] = "checkip.dyndns.com", update[] = "members.dyndns.org";
	memset(&DDNSClient,0,sizeof(DDNSClient));

	if (AppConfig.DynDNSEnable)
	{
		DDNSClient.CheckIPServer.szROM = checkip;
		DDNSClient.ROMPointers.CheckIPServer = 1;
		DDNSClient.CheckIPPort = 80;
		DDNSClient.UpdateServer.szROM = update;
		DDNSClient.ROMPointers.UpdateServer = 1;
		DDNSClient.UpdatePort = 80;
		DDNSClient.Username.szRAM = AppConfig.DynDNSUsername;
		DDNSClient.Password.szRAM = AppConfig.DynDNSPassword;
		DDNSClient.Host.szRAM = AppConfig.DynDNSHost;
	}
}

/*****************************************************************************/
//									     //
//		Menu Input Helper Function				     //
//									     //
/*****************************************************************************/
/* Returns: 0 = aborted, 1 = quit (q), 2 = reboot (r), 3 = exit (x), 4 = got input */
static int menu_get_input(ROM char *prompt)
{
	aborted = 0;
	
	while(!aborted)
	{
		printf(prompt);
		memset(cmdstr,0,sizeof(cmdstr));
		
		if (!myfgets(cmdstr,sizeof(cmdstr) - 1)) continue;
		
		if (!strchr(cmdstr,'!')) break;
	}
	
	if (aborted) return 0;
	
	if ((strchr(cmdstr,'Q')) || strchr(cmdstr,'q'))
	{
		CloseTelnetConsole();
		return 1;
	}
	
	if ((strchr(cmdstr,'R')) || strchr(cmdstr,'r'))
	{
		CloseTelnetConsole();
		log_prefix(); 
		printf(booting);
		RTCM_Reset();
		return 2; // Never reached
	}
	
	if ((strchr(cmdstr,'X')) || strchr(cmdstr,'x'))
	{
		return 3;
	}
	
	return 4; // Got input
}

/* Print common menu footer with navigation options */
	static void menu_print_footer(ROM char *menu_name)
{
	printf("\n99 - Save to EEPROM\n"
		"x  - Exit %s\n"
		"q  - Disconnect\n"
		"r - Reboot\n", menu_name);
	
	// Display cold power reboot timer warning if active
	if (cold_power_reboot_active && uptimer < cold_power_reboot_target)
	{
		DWORD remaining = (cold_power_reboot_target - uptimer) / 10;  // convert to seconds
		printf("\n*** Cold pwr reboot in %lu:%02lu ***\n", 
			remaining / 60, remaining % 60);
	}
	
	printf("\n");
	fflush(stdout);
}
/*****************************************************************************/
//									     //
//		IP Menu							     //
//									     //
/*****************************************************************************/
static void IPMenu()
{
	while(1) 
	{
		unsigned int i1,i2,i3,i4,x;
		BOOL bootok,ok;
		int sel;

		static ROMNOBEW char menu[] = "\nIP Menu\n\n" 
		"1  - IP (%d.%d.%d.%d)\n",
		menu1[] = 
		"2  - Mask (%d.%d.%d.%d)\n",
		menu2[] = 
		"3  - GW (%d.%d.%d.%d)\n",
		menu3[] = 
		"4  - DNS1 (%d.%d.%d.%d)\n",
		menu4[] = 
		"5  - DNS2 (%d.%d.%d.%d)\n",
		menu5[] = 
		"6  - DHCP (%d)\n"
		"7  - Port (%d)\n"
		"8  - User (%s)\n"
		"9  - Pass (%s)\n"
		"10 - DynDNS (%d)\n",
		menu6[] = 
		"11 - DynUser (%s)\n"
		"12 - DynPass (%s)\n"
		"13 - DynHost (%s)\n",
		menu7[] = 
		"14 - BootIP (%d.%d.%d.%d) (%s)\n"
		"15 - EthDpx [0=Half,1=Full] (%d)\n";

		ROM char *entsel = str_enter_sel;

		bootok = ((AppConfig.BootIPCheck == GetBootCS()));
		printf(menu,AppConfig.DefaultIPAddr.v[0],AppConfig.DefaultIPAddr.v[1],
			AppConfig.DefaultIPAddr.v[2],AppConfig.DefaultIPAddr.v[3]);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu1,AppConfig.DefaultMask.v[0],AppConfig.DefaultMask.v[1],
			AppConfig.DefaultMask.v[2],AppConfig.DefaultMask.v[3]);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu2,AppConfig.DefaultGateway.v[0],AppConfig.DefaultGateway.v[1],
			AppConfig.DefaultGateway.v[2],AppConfig.DefaultGateway.v[3]);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu3,AppConfig.DefaultPrimaryDNSServer.v[0],AppConfig.DefaultPrimaryDNSServer.v[1],
			AppConfig.DefaultPrimaryDNSServer.v[2],AppConfig.DefaultPrimaryDNSServer.v[3]);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu4,AppConfig.DefaultSecondaryDNSServer.v[0],AppConfig.DefaultSecondaryDNSServer.v[1],
			AppConfig.DefaultSecondaryDNSServer.v[2],AppConfig.DefaultSecondaryDNSServer.v[3]);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu5,AppConfig.Flags.bIsDHCPReallyEnabled,
			AppConfig.TelnetPort,AppConfig.TelnetUsername,AppConfig.TelnetPassword,AppConfig.DynDNSEnable);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu6,AppConfig.DynDNSUsername,AppConfig.DynDNSPassword,AppConfig.DynDNSHost);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu7,AppConfig.BootIPAddr.v[0],AppConfig.BootIPAddr.v[1],AppConfig.BootIPAddr.v[2],
			AppConfig.BootIPAddr.v[3],(bootok) ? "OK" : "BAD",AppConfig.EthFullDuplex);
		main_processing_loop();
		secondary_processing_loop();
		menu_print_footer("IP Menu");
		
		switch(menu_get_input(entsel))
		{
			case 0: continue; // aborted
			case 1: continue; // quit
			case 2: continue; // reboot (never reached)
			case 3: return;   // exit menu
		}

		
		printf(" \n");
		sel = atoi(cmdstr);

		if ((sel >= 1) && (sel <= 15))
		{
			printf(str_enter_newval);

			if (aborted) continue;

			if ((!myfgets(cmdstr,sizeof(cmdstr) - 1)) || (strlen(cmdstr) < 2))
			{
				printf(err_noentry_notchanged);
				continue;
			}

			if (aborted) continue;
		}
		ok = 0;
		switch(sel)
		{
			case 1: // Default IP address
				if ((sscanf(cmdstr,"%d.%d.%d.%d",&i1,&i2,&i3,&i4) == 4) &&
					(i1 < 256) && (i2 < 256) && (i3 < 256) && (i4 < 256))
				{
					AppConfig.DefaultIPAddr.v[0] = i1;
					AppConfig.DefaultIPAddr.v[1] = i2;
					AppConfig.DefaultIPAddr.v[2] = i3;
					AppConfig.DefaultIPAddr.v[3] = i4;
					ok = 1;
				}
				break;

			case 2: // Default Netmask
				if ((sscanf(cmdstr,"%d.%d.%d.%d",&i1,&i2,&i3,&i4) == 4) &&
					(i1 < 256) && (i2 < 256) && (i3 < 256) && (i4 < 256))
				{
					AppConfig.DefaultMask.v[0] = i1;
					AppConfig.DefaultMask.v[1] = i2;
					AppConfig.DefaultMask.v[2] = i3;
					AppConfig.DefaultMask.v[3] = i4;
					ok = 1;
				}
				break;

			case 3: // Default Gateway
				if ((sscanf(cmdstr,"%d.%d.%d.%d",&i1,&i2,&i3,&i4) == 4) &&
					(i1 < 256) && (i2 < 256) && (i3 < 256) && (i4 < 256))
				{
					AppConfig.DefaultGateway.v[0] = i1;
					AppConfig.DefaultGateway.v[1] = i2;
					AppConfig.DefaultGateway.v[2] = i3;
					AppConfig.DefaultGateway.v[3] = i4;
					ok = 1;
				}
				break;

			case 4: // Default Primary DNS
				if ((sscanf(cmdstr,"%d.%d.%d.%d",&i1,&i2,&i3,&i4) == 4) &&
					(i1 < 256) && (i2 < 256) && (i3 < 256) && (i4 < 256))
				{
					AppConfig.DefaultPrimaryDNSServer.v[0] = i1;
					AppConfig.DefaultPrimaryDNSServer.v[1] = i2;
					AppConfig.DefaultPrimaryDNSServer.v[2] = i3;
					AppConfig.DefaultPrimaryDNSServer.v[3] = i4;
					ok = 1;
				}
				break;

			case 5: // Default Secondary DNS
				if ((sscanf(cmdstr,"%d.%d.%d.%d",&i1,&i2,&i3,&i4) == 4) &&
					(i1 < 256) && (i2 < 256) && (i3 < 256) && (i4 < 256))
				{
					AppConfig.DefaultSecondaryDNSServer.v[0] = i1;
					AppConfig.DefaultSecondaryDNSServer.v[1] = i2;
					AppConfig.DefaultSecondaryDNSServer.v[2] = i3;
					AppConfig.DefaultSecondaryDNSServer.v[3] = i4;
					ok = 1;
				}
				break;

			case 6: // DHCP Enable
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 2))
				{
					AppConfig.Flags.bIsDHCPReallyEnabled = i1;
					ok = 1;
				}
				break;

			case 7: // Telnet Port
				if (sscanf(cmdstr,"%u",&i1) == 1) 
				{
					AppConfig.TelnetPort = i1;
					ok = 1;
				}
				break;

			case 8: // Telnet Username
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.TelnetUsername)))
				{
					cmdstr[x - 1] = 0;
					strcpy((char *)AppConfig.TelnetUsername,cmdstr);
					ok = 1;
				}
				break;

			case 9: // Telnet Password
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.TelnetPassword)))
				{
					cmdstr[x - 1] = 0;
					strcpy((char *)AppConfig.TelnetPassword,cmdstr);
					ok = 1;
				}
				break;

			case 10: // DynDNS Enable
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 2))
				{
					AppConfig.DynDNSEnable = i1;
					ok = 1;
					SetDynDNS();
				}
				break;

			case 11: // DynDNS Username
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.DynDNSUsername)))
				{
					cmdstr[x - 1] = 0;
					strcpy((char *)AppConfig.DynDNSUsername,cmdstr);
					ok = 1;
					SetDynDNS();
				}
				break;

			case 12: // DynDNS Password
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.DynDNSPassword)))
				{
					cmdstr[x - 1] = 0;
					strcpy((char *)AppConfig.DynDNSPassword,cmdstr);
					ok = 1;
					SetDynDNS();
				}
				break;

			case 13: // DynDNS Host
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.DynDNSHost)))
				{
					cmdstr[x - 1] = 0;
					strcpy((char *)AppConfig.DynDNSHost,cmdstr);
					ok = 1;
					SetDynDNS();
				}
				break;

			case 14: // BootLoader IP Address
				if ((sscanf(cmdstr,"%d.%d.%d.%d",&i1,&i2,&i3,&i4) == 4) &&
					(i1 < 256) && (i2 < 256) && (i3 < 256) && (i4 < 256))
				{
					AppConfig.BootIPAddr.v[0] = i1;
					AppConfig.BootIPAddr.v[1] = i2;
					AppConfig.BootIPAddr.v[2] = i3;
					AppConfig.BootIPAddr.v[3] = i4;
					AppConfig.BootIPCheck = GetBootCS();
					ok = 1;
				}
				break;

			case 15: // Ethernet Duplex Setting
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 2))
				{
					AppConfig.EthFullDuplex = i1;
					ok = 1;
				}
				break;

			case 99:
				SaveAppConfig();
				LOG_SYS("%s", saved);
				continue;

			default:
				printf(invalselection);
				continue;
		}
	}
}

/*****************************************************************************/
//									     //
//		Offline Menu						     //
//									     //
/*****************************************************************************/
static void OffLineMenu()
{

	while(1) 
	{
		unsigned int i1,x;
		BOOL ok;
		int sel;
		float f;

	static /*ROM*/ char menu[] = "\nOffline Menu\n\n" 
		"1  - Mode [0=OFF,1=Spx,2=Spx+Trg,3=Rpt] (%d)\n"
		"2  - CW Speed [x1/8000s] (%u)\n"
		"3  - PreCW [x1/8000s] (%u)\n"
		"4  - PostCW [x1/8000s] (%u)\n",
		menu1[] = 
		"5  - CW OffID (%s)\n"
		"6  - CW OnID (%s)\n"
		"7  - ID Per [x0.1s] (%u)\n"
		"8  - RptHang [x0.1s] (%u)\n",
		menu1a[] = 
		"9  - CTCSS [Hz] (%.1f)\n"
		"10 - CTCSS Lev (%d)\n"
		"11 - NoDeemp [0=Norm,1=Off] (%d)\n"
		"12 - Offline Delay [s] (%u)\n"
#if !defined(SMT_BOARD)
		"13 - AuxOut [0=ConnStatus,1=Hi,2=Lo] (%d)\n"
#endif
		;

		ROM char *entsel = str_enter_sel;

		printf(menu,AppConfig.FailMode,AppConfig.CWSpeed,AppConfig.CWBeforeTime,AppConfig.CWAfterTime);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu1,AppConfig.FailString,AppConfig.UnFailString,AppConfig.FailTime,AppConfig.HangTime);
		main_processing_loop();
		printf(menu1a,(double)AppConfig.CTCSSTone,AppConfig.CTCSSLevel,AppConfig.OffLineNoDeemp,AppConfig.OfflineDelay
#if !defined(SMT_BOARD)
		,AppConfig.AuxOutMode
#endif
		);
		main_processing_loop();
		secondary_processing_loop();
		menu_print_footer("Offline Menu");
		
		switch(menu_get_input(entsel))
		{
			case 0: continue; // aborted
			case 1: continue; // quit
			case 2: continue; // reboot (never reached)
			case 3: return;   // exit menu
		}

		
		printf(" \n");
		sel = atoi(cmdstr);

#if !defined(SMT_BOARD)
		if ((sel >= 1) && (sel <= 13))
#else
		if ((sel >= 1) && (sel <= 12))
#endif
		{
			printf(str_enter_newval);

			if (aborted) continue;

			if ((!myfgets(cmdstr,sizeof(cmdstr) - 1)) || 
				((strlen(cmdstr) < 2) && ((sel < 5) || (sel > 6))))
			{
				printf(err_noentry_notchanged);
				continue;
			}

			if (aborted) continue;
		}

		ok = 0;

		switch(sel)
		{
			case 1: // Fail Mode
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 3))
				{
					AppConfig.FailMode = i1;
					ok = 1;
				}
				break;

			case 2: // CW Speed
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 >= 100))
				{
					AppConfig.CWSpeed = i1;
					ok = 1;
				}
				break;

			case 3: // Pre-CW Delay
				if (sscanf(cmdstr,"%u",&i1) == 1) 
				{
					AppConfig.CWBeforeTime = i1;
					ok = 1;
				}
				break;

			case 4: // Post-CW Delay
				if (sscanf(cmdstr,"%u",&i1) == 1) 
				{
					AppConfig.CWAfterTime = i1;
					ok = 1;
				}
				break;

			case 5: // "Offline" (ID) String
				x = strlen(cmdstr);

				if ((x > 0) && (x < sizeof(AppConfig.FailString)))
				{
					cmdstr[x - 1] = 0;
					strupr(cmdstr);
					strcpy((char *)AppConfig.FailString,cmdstr);
					ok = 1;
				}
				break;

			case 6: // "Online" String
				x = strlen(cmdstr);

				if ((x > 0) && (x < sizeof(AppConfig.UnFailString)))
				{
					cmdstr[x - 1] = 0;
					strupr(cmdstr);
					strcpy((char *)AppConfig.UnFailString,cmdstr);
					ok = 1;
				}
				break;

			case 7: // Offline (ID) Period Time
				if (sscanf(cmdstr,"%u",&i1) == 1) 
				{
					AppConfig.FailTime = i1;
					ok = 1;
				}
				break;

			case 8: // Hang Time
				if (sscanf(cmdstr,"%u",&i1) == 1) 
				{
					AppConfig.HangTime = i1;
					ok = 1;
				}
				break;

			case 9: // CTCSS Tone
				f = atof(cmdstr);

				if ((f == 0.0) || ((f >= 60.0) && (f <= 300.0)))
				{
					AppConfig.CTCSSTone = f;
					SetCTCSSTone(AppConfig.CTCSSTone,AppConfig.CTCSSLevel);
					ok = 1;
				}
				break;

			case 10: // CTCSS Level
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 32767))
				{
					AppConfig.CTCSSLevel = i1;
					SetCTCSSTone(AppConfig.CTCSSTone,AppConfig.CTCSSLevel);
					ok = 1;
				}
				break;

			case 11: // DEEMP Override
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 1))
				{
					AppConfig.OffLineNoDeemp = i1;
					ok = 1;
				}
				break;

			case 12: // Offline Delay
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.OfflineDelay = i1;
					conn_status_offline_timer = 0;  // Reset timer on config change
					ok = 1;
				}
				break;

#if !defined(SMT_BOARD)
			case 13: // Aux Out Mode
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 2))
				{
					AppConfig.AuxOutMode = i1;
					ok = 1;
				}
				break;
#endif

			case 99:
				SaveAppConfig();
				LOG_SYS("%s", saved);
				continue;

			default:
				printf(invalselection);
				continue;
		}
		
		if (ok) printf(msg_changed_success);
		else printf(err_invalid_notchanged);
	}
}/*****************************************************************************/
//									     //
//		Squelch Menu						     //
//									     //
/*****************************************************************************/
static void SquelchMenu()
{

	while(1) 
	{
		unsigned int i1;
		BOOL ok;
		int sel;

	static /*ROM*/ char menu[] = "\nSquelch Menu\n\n" 
		"1  - Pot [0=HW,1=SW] (%d)\n"
		"2  - Sql Setting [1-1023] (%d)\n"
		"3  - Hyst [1-100] (%d)\n";

		ROM char *entsel = str_enter_sel;

		printf(menu,AppConfig.Sqpot,AppConfig.Squelch,AppConfig.Hysteresis);
		main_processing_loop();
		secondary_processing_loop();
		menu_print_footer("Squelch Menu");
		
		switch(menu_get_input(entsel))
		{
			case 0: continue; // aborted
			case 1: continue; // quit
			case 2: continue; // reboot (never reached)
			case 3: return;   // exit menu
		}

		
		printf(" \n");
		sel = atoi(cmdstr);

		if ((sel >= 1) && (sel <= 3))
		{
			printf(str_enter_newval);

			if (aborted) continue;

			if ((!myfgets(cmdstr,sizeof(cmdstr) - 1)) || 
				((strlen(cmdstr) < 2) && ((sel < 5) || (sel > 6))))
			{
				printf(err_noentry_notchanged);
				continue;
			}

			if (aborted) continue;
		}

		ok = 0;

		switch(sel)
		{
			case 1: // Hardware Pot or Software Pot
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 2))
				{
					AppConfig.Sqpot = i1;
					ok = 1;
				}
				break;

			case 2: // Squelch Setting
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 1023))
				{
					AppConfig.Squelch = i1;
					ok = 1;
				}
				break;

			case 3: // Hysteresis
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 100))
				{
					AppConfig.Hysteresis = i1;
					ok = 1;
				}
				break;

			case 99:
				SaveAppConfig();
				LOG_SYS("%s", saved);
				continue;

			default:
				printf(invalselection);
				continue;
		}
		
		if (ok) printf(msg_changed_success);
		else printf(err_invalid_notchanged);
	}
}

/*****************************************************************************/
//									     //
//		Auto Reboot Functions					     //
//									     //
/*****************************************************************************/

static void AutoRebootMenu()
{
	while(1) 
	{
		unsigned int i1;
		BOOL ok;
		int sel;

		printf(
			"\nAuto Reboot Menu - UTC: %s\n\n"

			"1 - Sched [0=Off,1=Daily,2=Wkly] (%u)\n"
			"2 - Day [0=Sun...6=Sat] (%u)\n"
			"3 - Hour (%u)\n"
			"4 - Min (%u)\n"
			"5 - Cold Power Reboot [m] (0=Off) (%u)\n\n",
			get_utc_time(),
			AppConfig.RebootMode, AppConfig.RebootDay, AppConfig.RebootHour, 
			AppConfig.RebootMinute, AppConfig.RebootColdPowerMins);
		main_processing_loop();
		menu_print_footer("Auto Reboot");
		
		switch(menu_get_input(str_enter_sel))
		{
			case 0: continue;
			case 1: continue;
			case 2: continue;
			case 3: return;
		}

		sel = atoi(cmdstr);

		if (sel >= 1 && sel <= 5)
		{
			printf(str_enter_newval);
			if (aborted || !myfgets(cmdstr,sizeof(cmdstr)-1) || strlen(cmdstr) < 2)
			{
				printf(err_noentry_notchanged);
				continue;
			}
		}

		ok = 0;

		switch(sel)
		{
			case 1:
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 2))
				{
					AppConfig.RebootMode = i1;
					ok = 1;
				}
				break;

			case 2:
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 6))
				{
					AppConfig.RebootDay = i1;
					ok = 1;
				}
				break;

			case 3:
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 23))
				{
					AppConfig.RebootHour = i1;
					ok = 1;
				}
				break;

			case 4:
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 59))
				{
					AppConfig.RebootMinute = i1;
					ok = 1;
				}
				break;

			case 5:
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.RebootColdPowerMins = i1;
					ok = 1;
					// Cancel active timer if changed to 0
					if (i1 == 0)
					{
						cold_power_reboot_active = FALSE;
						cold_power_reboot_target = 0;
					}
				}
				break;

			case 99:
				SaveAppConfig();
				LOG_SYS("%s", saved);
				continue;

			default:
				printf(invalselection);
				continue;
		}
		
		if (ok) printf(msg_changed_success);
		else printf(err_invalid_notchanged);
	}
}

/*****************************************************************************/
//									     //
//	MAIN Subroutine							     //
//									     //
/*****************************************************************************/

int main(void)
{

	WORD sel;
	time_t t;
	long mydiff,mydiff1;

	static /*ROM*/ char defwritten[] = "\nDefaults written\n",
			defdiode[] = "Diode cal written\n";
			
	// Save RCON value immediately at startup, then clear status bits per dsPIC manual
	saved_rcon = RCON;
	RCON &= 0x3F20;  // Clear all reset status bits: TRAPR(15), IOPWR(14), EXTR(7), SWR(6), WDTO(4), SLEEP(3), IDLE(2), BOR(1), POR(0)
	
	static /* ROM */ char menu1[] = "\n" 
		"1  - Serial# (%d) [MAC=%02X:%02X:%02X:%02X:%02X:%02X]\n"
		"2  - Client Pass (%s)\n"
		"3  - Local Port (%u)\n\n",
		menu2[] = 
		"4  - Server FQDN (%s)\n"
		"5  - Server Port (%u)\n"
		"6  - AltSvr FQDN (%s)\n"
		"7  - AltSvr Port (%u)\n"
		"8  - Host Pass (%s)\n\n",
		menu3[] = 
		"9  - GPS Baud (%lu)\n"
		"10 - GPS SerPol [0=Norm,1=Inv] (%d)\n"
		"11 - PPS Pol [0=Norm,1=Inv,2=OFF] (%d)\n"
		"12 - GPS Proto [0=NMEA,1=TSIP] (%d)\n"
		"13 - GPS Type [0=Norm,1=Tbolt] (%d)\n"
		"14 - GPS TimeOfs [s] (%lu)\n\n",
		menu4[] = 
		"15 - ExtCTCSS [0=Ign,1=Norm,2=Inv] (%d)\n"
		"16 - COR [0=Norm,1=Ign,2=NoRX] (%d)\n"
		"17 - Duplex3 [0=OFF,1-255x0.1s] (%u)\n"
		"18 - TxBuf Len [x1/8000s] (%d)\n"
		"19 - Launch Delay [x200ns,>0=ON] (%u)\n\n",
		menu5[] = 
		"20 - Debug Opts (%lu)\n"
#ifdef	DSPBEW
		"21 - DSP/BEW (%d)\n\n"
#else
		"21 - DSP/BEW [N/A]\n\n"
#endif
		"97 - RX Level\n"
		"98 - Status\n";

		ROM char *entsel = str_enter_sel;


	static ROM char oprdata[] = "\nVOTER Status\n\n"
		"= Sys =\n"
		"Ver:%s\n"
		"Ser:%u\n"
		"Up:%lu.%lus\n",
		curtimeis[] = "UTC:%s.%03lu\n",
		startuptime[] = "Start:%s\n\n",
	oprdata_net[] = 
		"= Net =\n"
		"MAC:%02X:%02X:%02X:%02X:%02X:%02X\n"
		"DHCP:%s\n"
		"IP:%d.%d.%d.%d\n"
		"Mask:%d.%d.%d.%d\n"
		"GW:%d.%d.%d.%d\n"
		"DNS1:%d.%d.%d.%d\n"
		"DNS2:%d.%d.%d.%d\n"
		"Port:%u\n\n",
	oprdata_gps[] = 
		"= GPS =\n"
		"Proto:%s\n"
		"State:%s\n"
		"Sync:%s\n"
		"Sats:%d\n"
		"PPSErr:%s\n\n",
	oprdata_gps1[] = "Ofs:%lds\n\n",
	oprdata_voter[] = 
		"= VOTER =\n"
		"Conn:%s\n"
		"Svr:%d.%d.%d.%d\n"
		"Port:%u\n",
	oprdata_radio[] = 
		"= Radio =\n"
		"COR:%s\n"
		"CTCSS:%s\n"
		"PTT:%s\n"
		"RSSI:%d\n"
		"Rate:%dsps\n"
		"Peak:%u\n"
		"TxBuf:%dms\n"
		"SqlGn:%d\n"
		"SqlDi:%d\n"
		"SqlLv:%d\n"
		"SqlHy:%d\n\n",
	oprdata_rcon[] =
		"= RCON =\n"
		"Val=0x%04X\n"
		"TRAP:%s IO:%s CM:%s\n"
		"EXTR:%s SW:%s WD:%s\n"
		"SLEP:%s IDL:%s BOR:%s POR:%s\n\n";


	portasave = 0;	
	filling_buffer = 0;
	fillindex = 0;
	filled = 0;
	time_filled = 0;
	connected = 0;
	lastrxtimer = 0;
	memclr((char *)audio_buf,FRAME_SIZE * 2);
	gps_bufindex = 0;
	TSIPwasdle = 0;
	gps_state = GPS_STATE_IDLE;
	gps_nsat = 0;
	gotpps = 0;
	gpssync = 0;
	ppscount = 0;
	rssi = 0;
	rssiheld = 0;
	adcother = 0;
	adcindex = 0;
	memclr(adcothers,sizeof(adcothers));
	sqlcount = 0;
	sql2 = 0;
	wascor = 0;
	lastcor = 0;
	vnoise32 = 0;
	option_flags = 0;
	txdrainindex = 0;
	last_drainindex = 0;
	memset(&lastrxtime,0,sizeof(lastrxtime));
	ptt = 0;
	myDHCPBindCount = 0xff;
	digest = 0;
	resp_digest = 0;
	their_challenge[0] = 0;
	ppstimer = 0;
	gpstimer = 0;
	gpsforcetimer = 0;
	attempttimer = 0;
	ppswarn = 0;
	gpswarn = 0;
	dwLastIP = 0;
	inread = 0;
	memset(&MyVoterAddr,0,sizeof(MyVoterAddr));
	memset(&LastVoterAddr,0,sizeof(LastVoterAddr));
	memset(&MyAltVoterAddr,0,sizeof(MyVoterAddr));
	memset(&CurVoterAddr,0,sizeof(CurVoterAddr));
	dnstimer = 0;
	alttimer = 0;
	dnsdone = 0;
	timing_time = 0;		// Time (whole secs) at beginning of next packet to be sent out
	timing_index = 0;		// Index at beginning of next packet to be sent out
	next_time = 0;			// Time (whole secs) samppled at begnning of current ADC frame
	next_index = 0;			// Index at beginning of current ADC frame
	samplecnt = 0;			// Index of ADC relative to PPS pulse
	last_adcsample = last_index = last_index1 = 2048;	// Last mulaw sample from ADC (so can be repeated if falls short)
	real_time = 0;			// Actual time (whole secs)
	last_samplecnt = 0;		// Last sample count (for display purposes)
	digest = 0;	
	resp_digest = 0;
	mydigest = 0;
	discfactor = 1000;
	discounterl = 0;
	discounteru = 0;
	amax = 0;
	amin = 0;
	apeak = 0;
	indisplay = 0;
	indipsw = 0;
	leddiag = 0;
	diagstate = 0;
	dec_valprev = 0;
	dec_index = 0;
	enc_valprev = 0;
	enc_index = 0;
	enc_prev_valprev = 0;
	enc_prev_index = 0;
	enc_lastdelta = 0;
	memset(dec_buffer,0,FRAME_SIZE);
	txseqno = 0;
	txseqno_ptt = 0;
	elketimer = 0;
	testidx = 0;
	testp = 0;
	errcnt = 0;
	measp = 0;
	measstr = 0;
	measidx = 0;
	diag_option_flags = 0;
	netisup = 0;
	gotbadmix = 0;
	telnet_echo = 0;
	termbuftimer = 0;
	termbufidx = 0;
	linkstate = 0;
	cwlen = 0;
	cwdat = 0;
	cwptr = 0;
	cwtimer = 0;
	cwidx = 0;
	cwtimer1 = 0;
	cwid_ptt_delay_timer = 0;
	connrep = 0;
	connfail = 0;
	ever_connected = 0;
	failtimer = 0;
	hangtimer = 0;
	dsecondtimer = 0;
	repeatit = 0;
	needburp = 0;
	tone_v1 = 0;
	tone_v2 = 0;
	tone_v3 = 0;
	tone_fac = 0;
	host_ptt = 0;
	hosttimedout = 0;
	altdns = 0;
	altchange = 0;
	althost = 0;
	lastalthost = 0;
	altconnected = 0;
	altchange = 0;
	altchange1 = 0;
	glasertimer = 0;
	conn_status_offline_timer = 0;
	uptimer = 0;
	pingtimer = 0;
	secondtimer = 0;
	missed = 0;
	misstimer = 0;
	misstimer1 = 0;
	memset(&last_rxpacket_time,0,sizeof(last_rxpacket_time));
	memset(&last_rxpacket_sys_time,0,sizeof(last_rxpacket_sys_time));
	last_rxpacket_index = 0;
	last_rxpacket_inbounds = 0;
	cold_power_reboot_target = 0;
	cold_power_reboot_active = FALSE;

	// Initialize application specific hardware
	InitializeBoard();

	RCONbits.SWDTEN = 0;

	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
	TickInit();

	SetLED(SYSLED,1);


	// Initialize Stack and application related NV variables into AppConfig.
	InitAppConfig();

	if ((!USE_PPS) || (!SIMULCAST_ENABLE)) 
	{
		DAC1CONbits.DACEN = 1;
		IEC4bits.DAC1LIE = 1;
	}

	// UART
#if defined(STACK_USE_UART)
	
	U1MODE = 0x8000;	// Set UARTEN.  Note: this must be done before setting UTXEN
	U1STA = 0x0400;		// UTXEN set

	#define CLOSEST_U1BRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE1)/16/BAUD_RATE1-1)
	#define BAUD_ACTUAL1 (GetPeripheralClock()/16/(CLOSEST_U1BRG_VALUE+1))

	#define BAUD_ERROR1 ((BAUD_ACTUAL1 > BAUD_RATE1) ? BAUD_ACTUAL1-BAUD_RATE1 : BAUD_RATE1-BAUD_ACTUAL1)
	#define BAUD_ERROR_PRECENT1	((BAUD_ERROR1*100+BAUD_RATE1/2)/BAUD_RATE1)
	#if (BAUD_ERROR_PRECENT1 > 3)
		#warning UART1 frequency error is worse than 3%
	#elif (BAUD_ERROR_PRECENT1 > 2)
		#warning UART1 frequency error is worse than 2%
	#endif
	
	U1BRG = CLOSEST_U1BRG_VALUE;
	U2MODE = 0x8000;	// Set UARTEN.  Note: this must be done before setting UTXEN
#if !defined(SMT_BOARD)
	U2MODEbits.URXINV = AppConfig.GPSPolarity ^ 1;
#else
	U2MODEbits.URXINV = AppConfig.GPSPolarity;
#endif

	U2STA = 0x0400;		// UTXEN set
#if !defined(SMT_BOARD)
	U2STAbits.UTXINV = AppConfig.GPSPolarity ^ 1;
#else
	U2STAbits.UTXINV = AppConfig.GPSPolarity;
#endif
	#define CLOSEST_U2BRG_VALUE ((GetPeripheralClock()+8ul*AppConfig.GPSBaudRate)/16/AppConfig.GPSBaudRate-1)
	U2BRG = CLOSEST_U2BRG_VALUE;

	InitUARTS();
#endif

	// Initiates board setup process if button is depressed on startup
	if(!INITIALIZE)
	{
		#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
		// Invalidate the EEPROM contents if BUTTON is held down for more than 4 seconds
		DWORD StartTime = TickGet();
		SetLED(SYSLED,1);

		while(!INITIALIZE)
		{
			if(TickGet() - StartTime > 4*TICK_SECOND)
				{
					#if defined(EEPROM_CS_TRIS)
					XEEBeginWrite(0x0000);
					XEEWrite(0xFF);
					XEEEndWrite();
					#elif defined(SPIFLASH_CS_TRIS)
					SPIFlashBeginWrite(0x0000);
					SPIFlashWrite(0xFF);
					#endif
	
					log_prefix(); printf(defwritten);
	
					if (!INITIALIZE_WVF) 
					{
						InitAppConfig();
						AppConfig.SqlDiode = adcothers[ADCDIODE];
						SaveAppConfig();
						log_prefix(); printf(defdiode);
					}

					SetLED(SYSLED,0);
	
					while((LONG)(TickGet() - StartTime) <= (LONG)(9*TICK_SECOND/2));
					SetLED(SYSLED,1);

					while(!INITIALIZE);
					RTCM_Reset();
					break;
				}
		}
#endif
	}

	// Initialize core stack layers (MAC, ARP, TCP, UDP) and
	// application modules (HTTP, SNMP, etc.)
	StackInit(AppConfig.EthFullDuplex);

	SetAudioSrc();

	init_squelch();

#ifdef	DSPBEW

#ifndef FFTTWIDCOEFFS_IN_PROGMEM					/* Generate TwiddleFactor Coefficients */
	TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);	/* We need to do this only once at start-up */
#endif

#endif // DSPBEW

	udpSocketUser = INVALID_UDP_SOCKET;

	sprintf(challenge,"%lu",GenerateRandomDWORD() % 1000000000ul);

	// Now that all items are initialized, begin the co-operative
	// multitasking loop.  This infinite loop will continuously 
	// execute all stack-related tasks, as well as your own
	// application's functions.  Custom functions should be added
	// at the end of this loop.

	// Note that this is a "co-operative mult-tasking" mechanism
	// where every task performs its tasks (whether all in one shot
	// or part of it) and returns so that other tasks can do their
	// job.
	// If a task needs very long time to do its job, it must be broken
	// down into smaller pieces so that other tasks can have CPU time.
	__builtin_nop();

	ClrWdt();
	RCONbits.SWDTEN = 1;
	ClrWdt();

	SetCTCSSTone(AppConfig.CTCSSTone,AppConfig.CTCSSLevel);

	LOG_SYS("FW v%s\n", VERSION);
	LOG_SYS("Serial=%04X, MAC=%02X:%02X:%02X:%02X:%02X:%02X\n", 
		AppConfig.SerialNumber,
		AppConfig.MyMACAddr.v[0], AppConfig.MyMACAddr.v[1], AppConfig.MyMACAddr.v[2],
		AppConfig.MyMACAddr.v[3], AppConfig.MyMACAddr.v[4], AppConfig.MyMACAddr.v[5]);

	if (sizeof(AppConfig) != 1016)
	{	
		DWORD d;
		printf("??? %d\n",sizeof(AppConfig));
		for(d = 0; d < 2000000; d++) ClrWdt();
		Reset();
	}

	memset(&AppConfig.Zeros,0,sizeof(AppConfig.Zeros));
	AppConfig.DebugLevel1 &= 0xffffff00;
	AppConfig.DebugLevel1 |= AppConfig.DebugLevel & 0xff;
	SaveAppConfig();

	if (AppConfig.Flags.bIsDHCPReallyEnabled)
		dwLastIP = AppConfig.MyIPAddr.Val;

	SetDynDNS();

	if (!AppConfig.Flags.bIsDHCPReallyEnabled)
		AppConfig.Flags.bInConfigMode = FALSE;

	// Check for cold power-up (POR or BOR) and start auto-reboot timer if enabled
	if (AppConfig.RebootColdPowerMins > 0 && (saved_rcon & 0x0003))
	{
		cold_power_reboot_active = TRUE;
		cold_power_reboot_target = (DWORD)AppConfig.RebootColdPowerMins * 600;  // uptimer ticks at 10Hz
		LOG_SYS("Cold reboot %um\n", AppConfig.RebootColdPowerMins);
	}

	while(1) 
	{
		char ok;
		unsigned int i1,x;
		unsigned long l;

		SetTxTone(0);

		if ((!netisup) && ((!AppConfig.Flags.bIsDHCPEnabled) || (!AppConfig.Flags.bInConfigMode)))
			netisup = 1;

		SetAudioSrc();
		printf(menu1,AppConfig.SerialNumber,AppConfig.MyMACAddr.v[0],AppConfig.MyMACAddr.v[1],AppConfig.MyMACAddr.v[2],
			AppConfig.MyMACAddr.v[3],AppConfig.MyMACAddr.v[4],AppConfig.MyMACAddr.v[5],
			AppConfig.Password,AppConfig.DefaultPort);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu2,AppConfig.VoterServerFQDN,AppConfig.VoterServerPort,AppConfig.AltVoterServerFQDN,
			AppConfig.AltVoterServerPort,AppConfig.HostPassword);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu3,AppConfig.GPSBaudRate,AppConfig.GPSPolarity,AppConfig.PPSPolarity,AppConfig.GPSProto,
			AppConfig.GPSTbolt,AppConfig.GPSOffset);
		main_processing_loop();
		secondary_processing_loop();
		printf(menu4,AppConfig.ExternalCTCSS,AppConfig.CORType,AppConfig.Duplex3,AppConfig.TxBufferLength,
			AppConfig.LaunchDelay);
		main_processing_loop();
		secondary_processing_loop();
#ifdef	DSPBEW
		printf(menu5,AppConfig.DebugLevel1,AppConfig.BEWMode);
#else
		printf(menu5,AppConfig.DebugLevel1);
#endif
		printf("99 - Save to EEPROM\n\n");
		printf("i  - IP Menu\n"
			"o  - Offline Menu\n"
			"s  - Squelch Menu\n"
			"a  - Auto Reboot Menu\n\n"
			"q  - Disconnect\n"
			"r  - Reboot\n");
		
		// Display cold power reboot timer warning if active
		if (cold_power_reboot_active && uptimer < cold_power_reboot_target)
		{
			DWORD remaining = (cold_power_reboot_target - uptimer) / 10;  // convert to seconds
			printf("\n*** Cold pwr reboot in %lu:%02lu ***\n", 
				remaining / 60, remaining % 60);
		}
		
		printf("\n");
		
		switch(menu_get_input(entsel))
		{
			case 0: continue; // aborted
			case 1: continue; // quit
			case 2: continue; // reboot (never reached)
		}

		if ((strchr(cmdstr,'A')) || strchr(cmdstr,'a'))
		{
			AutoRebootMenu();
			continue;
		}

		if ((strchr(cmdstr,'I')) || strchr(cmdstr,'i'))
		{
			IPMenu();
			continue;
		}

		if ((strchr(cmdstr,'O')) || strchr(cmdstr,'o'))
		{
			OffLineMenu();
			continue;
		}

		if ((strchr(cmdstr,'S')) || strchr(cmdstr,'s'))
		{
			SquelchMenu();
			continue;
		}
		
		sel = atoi(cmdstr);
#ifdef	DSPBEW
		if (((sel >= 1) && (sel <= 21)) || (sel == 11780) || (sel == 1103) || (sel == 1170))
#else
		if ((((sel >= 1) && (sel <= 21)) || (sel == 11780) || (sel == 1103) || (sel == 1170)) && (sel != 21))
#endif
		{
		/* If user selected Debug Level (20), print the bit descriptions first */
			if (sel == 20)
			{
				printf("Debug Options\n\n"
					"1  - RX/TX log\n"
					"2  - Stats\n"
					"16 - NoTOS\n"
					"32 - GPS\n"
					"64 - FixGPS\n\n"
					"Note: Bitmask; Sum values (e.g. 3=1+2)\n\n");
			}			printf(str_enter_newval);

			if (aborted) continue;

			if ((!myfgets(cmdstr,sizeof(cmdstr) - 1)) || ((strlen(cmdstr) < 2) && (sel != 6)))
			{
				printf(err_noentry_notchanged);
				continue;
			}

			if (aborted) continue;
		}

		ok = 0;

		switch(sel)
		{
			case 1: // Serial # (part of Mac Address)
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.SerialNumber = i1;
					ok = 1;
				}
				break;

			case 2: // VOTER Client Password
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.Password)))
				{
					cmdstr[x - 1] = 0;
					strcpy(AppConfig.Password,cmdstr);
					ok = 1;
				}
				break;

			case 3: // My Default Port (Local Port)
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.DefaultPort = i1;
					ok = 1;
				}
				break;

			case 4: // VOTER Server FQDN
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.VoterServerFQDN)))
				{
					cmdstr[x - 1] = 0;
					strcpy(AppConfig.VoterServerFQDN,cmdstr);
					ok = 1;
				}
				break;

			case 5: // Voter Server PORT
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.VoterServerPort = i1;
					ok = 1;
				}
				break;

			case 6: // Alt VOTER Server FQDN
				x = strlen(cmdstr);

				if ((x > 0) && (x < sizeof(AppConfig.VoterServerFQDN)))
				{
					cmdstr[x - 1] = 0;
					strcpy(AppConfig.AltVoterServerFQDN,cmdstr);
					ok = 1;
				}
				break;

			case 7: // Alt Voter Server PORT
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.AltVoterServerPort = i1;
					ok = 1;
				}
				break;

			case 8: // VOTER Server Password (Host Pass)
				x = strlen(cmdstr);

				if ((x > 2) && (x < sizeof(AppConfig.HostPassword)))
				{
					cmdstr[x - 1] = 0;
					strcpy(AppConfig.HostPassword,cmdstr);
					ok = 1;
				}
				break;

			case 9: // GPS Baud Rate
				if ((sscanf(cmdstr,"%lu",&l) == 1) && (l >= 300L) && (l <= 230400L))
				{
					AppConfig.GPSBaudRate = l;
					ok = 1;
				}
				break;

			case 10: // GPS Invert (SerPol)
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 2))
				{
					AppConfig.GPSPolarity = i1;
					ok = 1;
				}
				break;

			case 11: // PPS Invert (PPS Pol)
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 3))
				{
					AppConfig.PPSPolarity = i1;
					ok = 1;
				}
				break;

			case 12: // GPS Proto (GPS Type)
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 >= GPS_NMEA) && (i1 <= GPS_TSIP))
				{
					AppConfig.GPSProto = i1;
					ok = 1;
				}
				break;

			case 13: // GPS is a Thunderbolt (GPS Type)
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 < 2))
				{
					AppConfig.GPSTbolt = i1;
					ok = 1;
				}
				break;

			case 14: // GPS Time Offset (seconds)
				if ((sscanf(cmdstr,"%lu",&l) == 1) && (l <= 788400000UL))
				{
					AppConfig.GPSOffset = l;
					ok = 1;
				}
				break;

			case 15: // EXT CTCSS
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 2))
				{
					AppConfig.ExternalCTCSS = i1;
					ok = 1;
				}
				break;

			case 16: // COR Type
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 2))
				{
					AppConfig.CORType = i1;
					ok = 1;
				}
				break;
			case 17: // Duplex3 Hang Time
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 255))
				{
					AppConfig.Duplex3 = i1;
					ok = 1;
				}
				break;

			case 18: // Tx Buffer Length
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 >= 480) && (i1 <= MAX_BUFLEN))
				{
					AppConfig.TxBufferLength = i1;
					ok = 1;
				}
				break;

			case 19: // Launch Delay
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 600))
				{
					AppConfig.LaunchDelay = i1;
					ok = 1;
				}
				break;

			case 20: // Debug Level
				if (sscanf(cmdstr,"%lu",&l) == 1)
				{
					AppConfig.DebugLevel1 = l;
					AppConfig.DebugLevel = l & 0xff;
					ok = 1;
				}
				break;
#ifdef	DSPBEW
			case 21: // BEW Mode
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 2))
				{
					AppConfig.BEWMode = i1;
					ok = 1;
				}
				break;
#endif

		case 97: // Display RX Level Quasi-Graphically  
			printf(" \rRX Level:\n");
			indisplay = 1;
			myfgets(cmdstr,sizeof(cmdstr) - 1);
			indisplay = 0;
			continue;		case 98:
			{
				ROM char *gps_state_str, *gps_proto_str;
				int sql_level;
				
				// Map GPS state to human-readable string
				switch(gps_state)
				{
					case GPS_STATE_IDLE: gps_state_str = "IDLE"; break;
					case GPS_STATE_RECEIVED: gps_state_str = "RX"; break;
					case GPS_STATE_VALID: gps_state_str = "VALID"; break;
					case GPS_STATE_SYNCED: gps_state_str = "SYNCED"; break;
					default: gps_state_str = "??"; break;
				}
				
				// Map GPS protocol
				if (AppConfig.GPSProto == 0) gps_proto_str = "NMEA";
				else if (AppConfig.GPSProto == 1) gps_proto_str = "TSIP";
				else gps_proto_str = "?";
				
				sql_level = AppConfig.Sqpot ? AppConfig.Squelch : adcothers[ADCSQPOT];
				
				t = system_time.vtime_sec;
				
				// System Info Section
				printf(oprdata,VERSION,AppConfig.SerialNumber,uptimer / 10,uptimer % 10);
				strftime(cmdstr,sizeof(cmdstr) - 1,"%a  %b %d, %Y  %H:%M:%S",gmtime(&t));
				if (((gps_state == GPS_STATE_SYNCED) || (!USE_PPS)) && system_time.vtime_sec)
				{
					printf(curtimeis,cmdstr,(unsigned long)system_time.vtime_nsec/1000000L);
					// Calculate and display startup time by subtracting uptime from current GPS time
					if (uptimer > 0)
					{
						time_t startup_time = system_time.vtime_sec - (uptimer / 10);
						strftime(cmdstr,sizeof(cmdstr) - 1,"%a  %b %d, %Y  %H:%M:%S",gmtime(&startup_time));
						printf(startuptime,cmdstr);
					}
				}
				else
				{
					printf("\n");  // Just add blank line if no GPS time yet
				}
				main_processing_loop();
				secondary_processing_loop();
				
				// Network Section
				printf(oprdata_net,
					AppConfig.MyMACAddr.v[0],AppConfig.MyMACAddr.v[1],AppConfig.MyMACAddr.v[2],
					AppConfig.MyMACAddr.v[3],AppConfig.MyMACAddr.v[4],AppConfig.MyMACAddr.v[5],
					AppConfig.Flags.bIsDHCPReallyEnabled ? "Y" : "N",
					AppConfig.MyIPAddr.v[0],AppConfig.MyIPAddr.v[1],AppConfig.MyIPAddr.v[2],AppConfig.MyIPAddr.v[3],
					AppConfig.MyMask.v[0],AppConfig.MyMask.v[1],AppConfig.MyMask.v[2],AppConfig.MyMask.v[3],
					AppConfig.MyGateway.v[0],AppConfig.MyGateway.v[1],AppConfig.MyGateway.v[2],AppConfig.MyGateway.v[3],
					AppConfig.PrimaryDNSServer.v[0],AppConfig.PrimaryDNSServer.v[1],AppConfig.PrimaryDNSServer.v[2],AppConfig.PrimaryDNSServer.v[3],
					AppConfig.SecondaryDNSServer.v[0],AppConfig.SecondaryDNSServer.v[1],AppConfig.SecondaryDNSServer.v[2],AppConfig.SecondaryDNSServer.v[3],
					AppConfig.MyPort);
				main_processing_loop();
				secondary_processing_loop();
				
				// GPS Section
				printf(oprdata_gps,gps_proto_str,gps_state_str,
					gpssync ? "Y" : "N",
					gps_nsat,
					ppsx ? "Y" : "N");
				if (AppConfig.GPSOffset != 0)
					printf(oprdata_gps1,AppConfig.GPSOffset);
				main_processing_loop();
				secondary_processing_loop();
				
				// VOTER Host Server Section
				printf(oprdata_voter,connected ? "Y" : "N",
					CurVoterAddr.v[0],CurVoterAddr.v[1],CurVoterAddr.v[2],CurVoterAddr.v[3],
					AppConfig.VoterServerPort);
				mydiff = system_time.vtime_sec - last_rxpacket_sys_time.vtime_sec;
				mydiff *= 1000;
				mydiff1 = system_time.vtime_nsec - last_rxpacket_sys_time.vtime_nsec;
				mydiff1 /= 1000000;
				mydiff += mydiff1;
				printf("LastRx(Sys): %s, %ldms ago\n",logtime_p(&last_rxpacket_sys_time),mydiff);
				main_processing_loop();
				secondary_processing_loop();
				mydiff = last_rxpacket_sys_time.vtime_sec - last_rxpacket_time.vtime_sec;
				mydiff *= 1000;
				mydiff1 = last_rxpacket_sys_time.vtime_nsec - last_rxpacket_time.vtime_nsec;
				mydiff1 /= 1000000;
				mydiff += mydiff1;
				printf("LastRx(Time): %s, d=%ldms\n",logtime_p(&last_rxpacket_time),mydiff);
				main_processing_loop();
				secondary_processing_loop();
				printf("LastRx Idx: %ld, InBounds: %d\n\n",last_rxpacket_index,last_rxpacket_inbounds);
				main_processing_loop();
				secondary_processing_loop();
				
				// Radio Section
				printf(oprdata_radio,
					lastcor ? "Y" : "N",
					CTCSSIN ? "Y" : "N",
					ptt ? "Y" : "N",
					rssiheld,last_samplecnt,apeak,
					AppConfig.TxBufferLength >> 3,
					AppConfig.SqlNoiseGain,AppConfig.SqlDiode,sql_level,AppConfig.Hysteresis);
				main_processing_loop();
				secondary_processing_loop();
				
				// RCON Section
				printf(oprdata_rcon,
					saved_rcon,
					(saved_rcon & 0x8000) ? "Y" : "N",  // TRAPR(15)
					(saved_rcon & 0x4000) ? "Y" : "N",  // IOPWR(14)
					(saved_rcon & 0x0200) ? "Y" : "N",  // CM(9)
					(saved_rcon & 0x0080) ? "Y" : "N",  // EXTR(7)
					(saved_rcon & 0x0040) ? "Y" : "N",  // SWR(6)
					(saved_rcon & 0x0010) ? "Y" : "N",  // WDTO(4)
					(saved_rcon & 0x0008) ? "Y" : "N",  // SLEEP(3)
					(saved_rcon & 0x0004) ? "Y" : "N",  // IDLE(2)
					(saved_rcon & 0x0002) ? "Y" : "N",  // BOR(1)
					(saved_rcon & 0x0001) ? "Y" : "N");  // POR(0)
				main_processing_loop();
				secondary_processing_loop();
				
				printf(paktc);
				fflush(stdout);
				myfgets(cmdstr,sizeof(cmdstr) - 1);
			}
			continue;			case 99:
				SaveAppConfig();
				LOG_SYS("%s", saved);
				continue;

			case 111:
				printf("Elkes (11780): %lu, Glasers (1103): %u, Sawyer (1170): %d\n",
					AppConfig.Elkes,AppConfig.Glasers,AppConfig.Sawyer);
				main_processing_loop();
				secondary_processing_loop();
				printf(paktc);
				fflush(stdout);
				myfgets(cmdstr,sizeof(cmdstr) - 1);
				continue;

			case 11780: // Elkes
				if ((sscanf(cmdstr,"%lu",&l) == 1) && (l >=0))
				{
					AppConfig.Elkes = l * 9677ul;
					ok = 1;
				}
				break;

			case 1103: // Glasers
				if (sscanf(cmdstr,"%u",&i1) == 1)
				{
					AppConfig.Glasers = i1;
					if (glasertimer > i1) glasertimer = i1;
					ok = 1;
				}
				break;

			case 1170: // Sawyer Mode
				if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 1))
				{
					AppConfig.Sawyer = i1;
					ok = 1;
				}
				break;

			default:
				printf(invalselection);
				continue;
		}

		if (ok) printf(msg_changed_success);
		else printf(err_invalid_notchanged);
	}
}


/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
static void InitializeBoard(void)
{	

/*Setup the system clock. We are going to use the PLL in the PIC.
Fin is input frequency (from crystal)
Fosc is output frequency of PLL (System Clock)
Fcy is Fosc/2 (Instruction Clock)

Fosc = Fin ( M/(N1 * N2) )

M is PLLFBD, but PLLFBD is offset by 2, so 30 is actually 32
N1 is PLLPRE
N2 is PLLPOST
N1 and N2 are configured with CLKDIV
In this case, N1 = N2 =2

Therefore, with a 9.6MHz crystal (Fin), Fosc = 9.6 ( 32 / 4 ) = 76.8MHz and 
Fcy is then 38.4MHz. Fvco is before PLLPOST, so Fvco = 153.6MHz.

For the DAC, it uses 256x oversampling, and it's clock will be Fosc. It is 
configured (below) for divide by 75. So, our sample rate becomes 
(153.6MHz / 256) / 75 = 8000 samples/sec, aka 8kHz sampling. ***This is 
critical for the ADPCM/uLAW encode/decode.*** This limits the available 
oscillators we can use, since we need to be able to configure the PLL to 
give us Fosc of 153.6MHz. Note, 9.8304MHz SHOULD also work, with the correct 
PLL settings... this is aka CDMA 8x Chip in ex-CDMA GPSDO's... just 'sayin. */

	// Multiplier (M) factor for the PLL ***offset by 2*** so it is really 32
	PLLFBD = 30;
	// N1=Fxtal/2, N2=Fvco/2
	CLKDIV = 0x0000;
	// Primary OSC is Clock Source, Divide by 1, AOSC disabled, 
	// PLL output (Fvco) provides source for Aux Clock Divider
	ACLKCON = 0x780;


/* Timer 3 is used for it's special feature to be able to trigger 
an ADC conversion. TMR3 always generates the ADC interrupt, the 
ADC must be configured to use it.

The divide by 8 prescaler makes each timer tick:
1/(Fcy/8) = 1/(38.4MHz/8) = 0.208uSec

So, every 300*0.208uSec = 62.5uSec, which is the sample time for 
the ADC, after which we will trigger an ADC conversion.

ADC conversion time is calculated as:
Tcy = 1/Fcy = 1/38.4MHz = 0.026042uS
Tad = 4*Tcy (configured below) = 0.1042uS
Tconv = 14*Tad = 1.458uS for 12-bit mode
*/
	// Turn off interrupts while we configure them.
	DISABLE_INTERRUPTS();

	// Timer 3, continue in idle mode, gating disabled, 
	// divide Fcy by 8
	T3CON = 0x10;
	// Set to period to 299 + 1 (total 62.5uSec)
	PR3 = 299;
	// Turn it on
	T3CONbits.TON = 1;

// Configure our ADC pins
#if defined(SMT_BOARD)
	// Enable AN0-AN3 as analog
	AD1PCFGL = 0xFFF0; 
#else
	// Enable AN0, AN2-AN4 as analog
	// AN0 RX Audio, AN2 Noise Voltage (RSSI)
	// AN3 SQ Ctrl Pos, AN4 Diode (Temp Comp) Votlage
	AD1PCFGL = 0xFFE2; 
#endif
	
	// ADC is operating, continue operation in idle mode
	// DMA written in scatter/gather mode
	// 12-bit mode, Unsigned Int output (0x0000-0x0FFF)
	// Use TMR3 to end sampling and start conversion
	// Sample multiple channels individually in sequence
	// Sampling begins again immediately after last conversion
	AD1CON1 = 0x8444;
	// ADREF+ is AVdd and ADREF- is AVss
	// Do not scan inputs
	// Conversion CH0, since we are in 12-bit mode
	// Start buffer fill at address 0x0, use channel input selects for Sample A
	AD1CON2 = 0x0;
	// ADC Clock from system clock (Fosc/2 = Fcy)
	// (Clock calculations above)
	// Auto Sample Time is 16*Tad... but this isn't used since TMR3 
	// is controlling sample time
	// ADC Conversion Clock Select (multiplier) = (3 + 1) Tcy = Tad
	AD1CON3 = 0x1003;
	// Turn on the ADC module (already turned on)
	AD1CON1bits.ADON = 1;

	// Clear the AD1 flag
	IFS0bits.AD1IF = 0;
	// Enable AD1 interrupts
	IEC0bits.AD1IE = 1;
	// Set the interrupt priority to 6
	IPC3bits.AD1IP = 6;

// Configure the DAC (TX audio output)
	DAC1DFLT = 0;
	DAC1STAT = 0x8000;
	DAC1STATbits.LITYPE = 0;
	DAC1CON = 0x1100 + 74;		// Divide by 75 for 8K Samples/sec
	IFS4bits.DAC1LIF = 0;
//	IEC4bits.DAC1LIE = 1;
	IPC19bits.DAC1LIP = 6;

#if defined(SMT_BOARD)
	PORTA=0;	
	PORTB=0x3c00;
	PORTC=7;	
	// RA4 is CN0/PPS Pulse, RA7-CTCSS, RA8-RA10 jumpers
	TRISA = 0xFFFF;	 
	// RB0-1 are Analog, 
	// RB2-3 are audio select, 
	// RB4 is PTT, RB5-6 are Programming Pins, 
	// RB7 is INT0 (Ethenet INT)
	// RB8 is Test Pin, RB9 is JP11, 
	// RB10-13 are LEDs, RB14-15 are DAC outputs
	TRISB = 0x0283;// 0x02E3;	
	// RC0-RC2 are CS pins, RC4 is RP20/SDO, 
	// RC5 is RP21/SCK, RC7 is RP23/U1TX, RC9 is RP25/U2TX
	TRISC = 0xFD48;

	__builtin_write_OSCCONL(OSCCON & ~0x40); //clear the bit 6 of OSCCONL to unlock pin re-map
	_U1RXR = 22;	// RP22 is UART1 RX
	_RP23R = 3;	// RP23 is UART1 TX (U1TX) 3
	_U2RXR = 24;	// RP24 is UART2 RX
	_RP25R = 5;	// RP25 is UART2 TX (U2TX) 5 
	_SDI1R = 19;	// RP19 is SPI1 MISO
	_RP21R = 8;	// RP21 is SPI1 CLK (SCK1OUT) 8
	_RP20R = 7;	// RP20 is SPI1 MOSI (SDO1) 7
	__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock pin re-map
#else
	PORTA=0;	// Initialize LED pin data to off state
	PORTB=0;	// Initialize LED pin data to off state
	// RA4 is CN0/PPS Pulse
	TRISA = 0xFFFD;	// RA1 is Test Bit -- Set to 0xFFF5 for RA1/RA3 Test Bits
	// RB0-2 are Analog, RB3-4 are SPI select, 
	// RB5-6 are Programming pins, RB7 is INT0 (Ethenet INT), 
	// RB8 is RP8/SCK, RB9 is RP9/SDO, RB10 is RP10/SDI, 
	// RB11 is RP11/U1TX, RB12 is RP12/U1RX, RB13 is RP13/U2RX, 
	// RB14-15 are DAC outputs
	TRISB = 0x3487;	

	__builtin_write_OSCCONL(OSCCON & ~0x40); //clear the bit 6 of OSCCONL to unlock pin re-map
	_U1RXR = 12;	// RP12 is UART1 RX
	_RP11R = 3;	// RP11 is UART1 TX (U1TX)
	_U2RXR = 13;	// RP13 is UART2 RX
	_SDI1R = 10;	// RP10 is SPI1 MISO
	_RP8R = 8;	// RP8 is SPI1 CLK (SCK1OUT) 8
	_RP9R = 7;	// RP9 is SPI1 MOSI (SDO1) 7
	__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock pin re-map

	IOExpInit();

#if !defined(SMT_BOARD)
	/* Ensure GPB6 (bit 6) is HIGH at startup (GPS Reset line idle HIGH) */
	IOExpOutB |= 0x40; /* GPB6 */
	IOExp_Write(IOEXP_OLATB, IOExpOutB);
#endif
#endif

#if defined(SPIRAM_CS_TRIS)
	SPIRAMInit();
#endif
#if defined(EEPROM_CS_TRIS)
	XEEInit();
#endif
#if defined(SPIFLASH_CS_TRIS)
	SPIFlashInit();
#endif

	CNEN1bits.CN0IE = 1;	// Change Notification CN0 interrupt enable
	IEC1bits.CNIE = 1;	// Enable CN interrupt
	IPC4bits.CNIP = 6;	// Set interrupt priority to 6

	INTCON1bits.NSTDIS = 0;

	ENABLE_INTERRUPTS();
}

/*********************************************************************
 * Function:        void InitAppConfig(void)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           None
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
// MAC Address Serialization using a MPLAB PM3 Programmer and 
// Serialized Quick Turn Programming (SQTP). 
// The advantage of using SQTP for programming the MAC Address is it
// allows you to auto-increment the MAC address without recompiling 
// the code for each unit.  To use SQTP, the MAC address must be fixed
// at a specific location in program memory.  Uncomment these two pragmas
// that locate the MAC address at 0x1FFF0.  Syntax below is for MPLAB C 
// Compiler for PIC18 MCUs. Syntax will vary for other compilers.
//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
	memset(&AppConfig,0,sizeof(AppConfig));
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bIsDHCPReallyEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
	AppConfig.SerialNumber = 1234;
	AppConfig.DefaultIPAddr.v[0] = 192;
	AppConfig.DefaultIPAddr.v[1] = 168;
	AppConfig.DefaultIPAddr.v[2] = 1;
	AppConfig.DefaultIPAddr.v[3] = 10;
	AppConfig.BootIPAddr.v[0] = 192;
	AppConfig.BootIPAddr.v[1] = 168;
	AppConfig.BootIPAddr.v[2] = 1;
	AppConfig.BootIPAddr.v[3] = 11;
	AppConfig.BootIPCheck = GetBootCS();
	AppConfig.DefaultMask.v[0] = 255;
	AppConfig.DefaultMask.v[1] = 255;
	AppConfig.DefaultMask.v[2] = 255;
	AppConfig.DefaultMask.v[3] = 0;
	AppConfig.DefaultGateway.v[0] = 192;
	AppConfig.DefaultGateway.v[1] = 168;
	AppConfig.DefaultGateway.v[2] = 1;
	AppConfig.DefaultGateway.v[3] = 1;
	AppConfig.DefaultPrimaryDNSServer.v[0] = 192;
	AppConfig.DefaultPrimaryDNSServer.v[1] = 168;
	AppConfig.DefaultPrimaryDNSServer.v[2] = 1;
	AppConfig.DefaultPrimaryDNSServer.v[3] = 1;
	AppConfig.DefaultSecondaryDNSServer.v[0] = 0;
	AppConfig.DefaultSecondaryDNSServer.v[1] = 0;
	AppConfig.DefaultSecondaryDNSServer.v[2] = 0;
	AppConfig.DefaultSecondaryDNSServer.v[3] = 0;

	AppConfig.TxBufferLength = DEFAULT_TX_BUFFER_LENGTH;
	AppConfig.VoterServerPort = 1667;
	AppConfig.GPSBaudRate = 9600;
	strcpy(AppConfig.Password,"radios");
	strcpy(AppConfig.HostPassword,"BLAH");
	strcpy(AppConfig.VoterServerFQDN,"voter-demo.allstarlink.org");
	AppConfig.TelnetPort = 23;
	strcpy((char *)AppConfig.TelnetUsername,"admin");
	strcpy((char *)AppConfig.TelnetPassword,"radios");
	AppConfig.DynDNSEnable = 0;
	strcpy((char *)AppConfig.DynDNSUsername,"wb6nil");
	strcpy((char *)AppConfig.DynDNSPassword,"radios42");
	strcpy((char *)AppConfig.DynDNSHost,"voter-test.dyndns.org");
	AppConfig.CWSpeed = 400;
	AppConfig.CWBeforeTime = 4000;
	AppConfig.CWAfterTime = 4000;
	strcpy((char *)AppConfig.FailString,"OFF LINE");
	strcpy((char *)AppConfig.UnFailString,"OK");
	AppConfig.HangTime = 15;
	AppConfig.CTCSSTone = 0.0;
	AppConfig.CTCSSLevel = 3000;
	AppConfig.PPSPolarity = 0;
	AppConfig.GPSTbolt = 0;
	AppConfig.GPSOffset = 0;
	AppConfig.Squelch = 400;
	AppConfig.Hysteresis = 24;
	AppConfig.Sqpot = 0;
	AppConfig.AuxOutMode = 0;  // Default: auto mode (connection status)
	AppConfig.OfflineDelay = 0;  // Default: no delay (immediate offline indication)

	#if defined(EEPROM_CS_TRIS)
	{
		BYTE c;
		
		// When a record is saved, first byte is written as 0x60 to indicate
		// that a valid record was saved.  Note that older stack versions
		// used 0x57.  This change has been made to so old EEPROM contents
		// will get overwritten.  The AppConfig() structure has been changed,
		// resulting in parameter misalignment if still using old EEPROM
		// contents.
		XEEReadArray(0x0000, &c, 1);/*SaveAppConfig(); */

		if(c == 0x60u)
			XEEReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#elif defined(SPIFLASH_CS_TRIS)
	{
		BYTE c;
		
		SPIFlashReadArray(0x0000, &c, 1);
		if(c == 0x60u)
			SPIFlashReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#endif

	AppConfig.MyIPAddr = AppConfig.DefaultIPAddr;
	AppConfig.MyMask = AppConfig.DefaultMask;
	AppConfig.MyGateway = AppConfig.DefaultGateway;
	AppConfig.PrimaryDNSServer = AppConfig.DefaultPrimaryDNSServer;
	AppConfig.SecondaryDNSServer = AppConfig.DefaultSecondaryDNSServer;
	AppConfig.MyPort = AppConfig.VoterServerPort;
	if (AppConfig.DefaultPort) AppConfig.MyPort = AppConfig.DefaultPort;
	AppConfig.MyMACAddr.v[5] = AppConfig.SerialNumber & 0xff;
	AppConfig.MyMACAddr.v[4] = AppConfig.SerialNumber >> 8;
	AppConfig.Flags.bIsDHCPEnabled = TRUE;

	if (AppConfig.AltVoterServerFQDN[0] == -1)
	{
		memset(AppConfig.AltVoterServerFQDN,0,sizeof(AppConfig.AltVoterServerFQDN));
		AppConfig.AltVoterServerPort = 0;
	}
}

#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
void SaveAppConfig(void)
{
	#if defined(EEPROM_CS_TRIS)
	XEEBeginWrite(0x0000);
	XEEWrite(0x60);
	XEEWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
	#else
	SPIFlashBeginWrite(0x0000);
	SPIFlashWrite(0x60);
	SPIFlashWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
	#endif
}
#endif
