/*********************************************************************
 *
 *	Telnet Server
 *  Module for Microchip TCP/IP Stack
 *	 -Provides Telnet services on TCP port 23
 *	 -Reference: RFC 854
 *
 *********************************************************************
 * FileName:        Telnet.c
 * Dependencies:    TCP
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *					Microchip C30 v3.12 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2009 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date    Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder     9/12/06	Original
 ********************************************************************/
#define __TELNET_C

#include "TCPIPConfig.h"

#if defined(STACK_USE_TELNET_SERVER)

#include "TCPIP Stack/TCPIP.h"
#include "UART.h"

// Set up configuration parameter defaults if not overridden in 
// TCPIPConfig.h
#if !defined(TELNETS_PORT)	
    // SSL Secured Telnet port (ignored if STACK_USE_SSL_SERVER is undefined)
	#define TELNETS_PORT		992	
#endif
#if !defined(MAX_TELNET_CONNECTIONS)
    // Maximum number of Telnet connections
	#define MAX_TELNET_CONNECTIONS	(3u)
#endif

#define TELNET_PORT AppConfig.TelnetPort
#define	TELNET_USERNAME AppConfig.TelnetUsername
#define	TELNET_PASSWORD AppConfig.TelnetPassword

#define	MAXTERMBUF 100  // Make sure this is <= Telnet TX FIFO size!!

// Login/auth strings
static ROM BYTE strUsername[]       = "Username: ";
// DO Suppress Local Echo (stop telnet client from printing typed characters)
static ROM BYTE strPassword[]       = "Password: \xff\xfd\x2d";
static ROM BYTE strAccessDenied[]   = "\r\nAccess denied\r\n\r\n";
static ROM BYTE strAuthenticated[]  = "\rLogged in successfully...\r\n\r\n";

									  
extern BYTE AN0String[8];

/*********************************************************************
 * Function:        void TelnetTask(void)
 *
 * PreCondition:    Stack is initialized()
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Performs Telnet Server related tasks.  Contains
 *                  the Telnet state machine and state tracking
 *                  variables.
 *
 * Note:            None
 ********************************************************************/

enum
{
	SM_HOME = 0,
	SM_PRINT_BANNER,     // Print initial banner and enter streaming mode
	SM_STREAM,           // Stream console without authentication
	SM_GET_LOGIN,        // Prompted for username
	SM_GET_PASSWORD,     // Prompted for password
	SM_GET_PASSWORD_BAD_LOGIN,
	SM_AUTHENTICATED     // Logged in; menu input allowed
} TelnetState;
static TCP_SOCKET hTelnetSockets[MAX_TELNET_CONNECTIONS];
static BYTE vTelnetStates[MAX_TELNET_CONNECTIONS];
static BOOL bInitialized = FALSE;

static BYTE termbuf[MAXTERMBUF];
extern WORD termbufidx;
extern WORD termbuftimer;
// VERSION string from main firmware
extern char VERSION[];

// Simple helpers for redaction
static inline BOOL is_digit(BYTE c) { return (c >= '0') && (c <= '9'); }

// Redact IPv4 dotted quads.
// - If in the form " (IPv4)", remove the entire parenthesized block including the leading space
// - If in the form "(IPv4)", remove the entire parenthesized block
// - Otherwise, replace bare IPv4 tokens with "[redacted]"
// Returns number of bytes written to out (<= cap).
static WORD redact_ips(const BYTE* s, WORD len, BYTE* out, WORD cap)
{
	WORD i = 0, o = 0;
	const BYTE repl[] = "[redacted]"; // length 10
	while(i < len && o < cap)
	{

		// Case 1: " (IPv4)" — remove whole block (and the leading space)
		if(s[i] == ' ' && (i + 1) < len && s[i+1] == '(')
		{
			WORD j = i + 2; // start after ' ('
			BYTE parts;
			WORD k = j;
			for(parts = 0; parts < 4; parts++)
			{
				BYTE dcnt = 0;
				if(k >= len || !is_digit(s[k])) { parts = 0xFF; break; }
				while(k < len && is_digit(s[k]) && dcnt < 3) { k++; dcnt++; }
				if(parts < 3)
				{
					if(k >= len || s[k] != '.') { parts = 0xFF; break; }
					k++; // consume '.'
				}
			}
			if(parts == 4 && k < len && s[k] == ')')
			{
				// Drop from space before '(' through ')'
				i = k + 1;
				continue;
			}
		}

		// Case 2: "(IPv4)" — remove whole parenthesized block
		if(s[i] == '(')
		{
			WORD j = i + 1; // start after '('
			BYTE parts;
			WORD k = j;
			for(parts = 0; parts < 4; parts++)
			{
				BYTE dcnt = 0;
				if(k >= len || !is_digit(s[k])) { parts = 0xFF; break; }
				while(k < len && is_digit(s[k]) && dcnt < 3) { k++; dcnt++; }
				if(parts < 3)
				{
					if(k >= len || s[k] != '.') { parts = 0xFF; break; }
					k++; // consume '.'
				}
			}
			if(parts == 4 && k < len && s[k] == ')')
			{
				// Drop from '(' through ')'
				i = k + 1;
				continue;
			}
		}

		// Case 3: Bare IPv4 — replace with [redacted]
		if(is_digit(s[i]))
		{
			// Try to match d{1,3}\.(d{1,3}\.){2}d{1,3}
			BYTE parts = 0;
			WORD j = i;
			for(parts = 0; parts < 4; parts++)
			{
				BYTE dcnt = 0;
				if(j >= len || !is_digit(s[j])) { parts = 0xFF; break; }
				while(j < len && is_digit(s[j]) && dcnt < 3) { j++; dcnt++; }
				// For first 3 parts, require a dot
				if(parts < 3)
				{
					if(j >= len || s[j] != '.') { parts = 0xFF; break; }
					j++; // consume '.'
				}
			}
			if(parts == 4)
			{
				// Matched an IPv4-like pattern; ensure token boundary (next char not digit)
				if(j >= len || !is_digit(s[j]))
				{
					// Replace with [redacted]
					BYTE k;
					for(k = 0; k < sizeof(repl)-1 && o < cap; k++)
						out[o++] = repl[k];
					i = j;
					continue;
				}
			}
		}
		// No match; copy one byte
		out[o++] = s[i++];
	}
	return o;
}

void TelnetTask(void)
{
	BYTE		vTelnetSession;
	WORD		w, w2;
	TCP_SOCKET	MySocket;
	char outstr[96];


	// Perform one time initialization on power up
	if(!bInitialized)
	{
		for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
		{
			hTelnetSockets[vTelnetSession] = INVALID_SOCKET;
			vTelnetStates[vTelnetSession] = SM_HOME;
		}
		bInitialized = TRUE;
	}

	// Loop through each telnet session and process state changes and TX/RX data
	for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
	{
		// Load up static state information for this session
		MySocket = hTelnetSockets[vTelnetSession];
		TelnetState = vTelnetStates[vTelnetSession];

		// Reset our state if the remote client disconnected from us
		if(MySocket != INVALID_SOCKET)
		{
			if(TCPWasReset(MySocket))
				TelnetState = SM_PRINT_BANNER;
		}

		// Handle session state
		switch(TelnetState)
		{
			case SM_HOME:
				// Connect a socket to the remote TCP server
				MySocket = TCPOpen(0, TCP_OPEN_SERVER, TELNET_PORT, TCP_PURPOSE_TELNET);
				
				// Abort operation if no TCP socket of type TCP_PURPOSE_TELNET is available
				// If this ever happens, you need to go add one to TCPIPConfig.h
				if(MySocket == INVALID_SOCKET)
					break;
	
				// Open an SSL listener if SSL server support is enabled
				#if defined(STACK_USE_SSL_SERVER)
					TCPAddSSLListener(MySocket, TELNETS_PORT);
				#endif
	
				TelnetState = SM_PRINT_BANNER;
				break;

			case SM_PRINT_BANNER:
				#if defined(STACK_USE_SSL_SERVER)
					// Reject unsecured connections if TELNET_REJECT_UNSECURED is defined
					#if defined(TELNET_REJECT_UNSECURED)
						if(!TCPIsSSL(MySocket))
						{
							if(TCPIsConnected(MySocket))
							{
								TCPDisconnect(MySocket);
								TCPDisconnect(MySocket);
								break;
							}	
						}
					#endif
						
					// Don't attempt to transmit anything if we are still handshaking.
					if(TCPSSLIsHandshaking(MySocket))
						break;
				#endif

				// Print new banner: VOTER #<serial> - Version <VERSION> + live console hint
				sprintf(outstr, "\r\n\nVOTER #%d - Version %s\r\n<live console; press Enter to log in>\r\n\n", AppConfig.SerialNumber, VERSION);
			
				// Make certain the socket can be written to
				if(TCPIsPutReady(MySocket) < strlen(outstr))
					break;
				
				// Place the application protocol data into the transmit buffer.
				TCPPutString(MySocket, (BYTE *)outstr);
	
				// Send the packet
				TCPFlush(MySocket);
				TelnetState = SM_STREAM;
	
			case SM_STREAM:
				// In streaming mode, only prompt for login when user presses Enter
				w = TCPFind(MySocket, '\n', 0, FALSE);
				if(w == 0xFFFFu)
					break;
				// Consume the line (including CR/LF)
				TCPGetArray(MySocket, NULL, w + 1);
				// Prompt for username now
				if(TCPIsPutReady(MySocket) < strlenpgm((ROM char*)strUsername))
					break;
				TCPPutROMString(MySocket, strUsername);
				TCPFlush(MySocket);
				TelnetState = SM_GET_LOGIN;
				break;

			case SM_GET_LOGIN:
				// Make sure we can put the password prompt
				if(TCPIsPutReady(MySocket) < strlenpgm((ROM char*)strPassword))
					break;
				// See if the user pressed return
				w = TCPFind(MySocket, '\n', 0, FALSE);
				if(w == 0xFFFFu)
				{
					if(TCPGetRxFIFOFree(MySocket) == 0u)
					{
						TCPPutROMString(MySocket, (ROM BYTE*)"\r\nToo much data.\r\n");
						// Return to streaming instead of disconnecting
						TelnetState = SM_STREAM;
					}
					break;
				}
				// Search for the username -- case insensitive
				w2 = TCPFindArray(MySocket, TELNET_USERNAME, strlen((char*)TELNET_USERNAME), 0, TRUE);
				if((w2 < 0) || !((w2 == ((w - strlen((char *)TELNET_USERNAME)) - 1)) || (w2 == (w - strlen((char *)TELNET_USERNAME)))))
				{
					// Did not find the username, but let's pretend we did so we don't leak the user name validity
					TelnetState = SM_GET_PASSWORD_BAD_LOGIN;	
				}
				else
				{
					TelnetState = SM_GET_PASSWORD;
				}
				// Username verified (or not), throw this line of data away
				TCPGetArray(MySocket, NULL, w + 1);
				// Print the password prompt
				TCPPutROMString(MySocket, strPassword);
				TCPFlush(MySocket);
				break;
	
			case SM_GET_PASSWORD:
			case SM_GET_PASSWORD_BAD_LOGIN:
				// Make sure we can put the authenticated prompt
				if(TCPIsPutReady(MySocket) < strlenpgm((ROM char*)strAuthenticated))
					break;
	
				// See if the user pressed return
				w = TCPFind(MySocket, '\n', 0, FALSE);
				if(w == 0xFFFFu)
				{
					if(TCPGetRxFIFOFree(MySocket) == 0u)
					{
						TCPPutROMString(MySocket, (ROM BYTE*)"Too much data.\r\n");
						TCPDisconnect(MySocket);
					}
	
					break;
				}
	
				// Search for the password -- case sensitive
				w2 = TCPFindArray(MySocket, TELNET_PASSWORD, strlen((char *)TELNET_PASSWORD), 0, FALSE);
				if((w2 != 3u) || !(((strlen((char *)TELNET_PASSWORD) == w-4)) || ((strlen((char *)TELNET_PASSWORD) == w-3)))
					|| (TelnetState == SM_GET_PASSWORD_BAD_LOGIN))
				{
					// Did not find the password - do not disconnect, return to streaming
					TelnetState = SM_STREAM;	
					TCPPutROMString(MySocket, strAccessDenied);
					TCPFlush(MySocket);
					break;
				}
	
				// Password verified, throw this line of data away
				TCPGetArray(MySocket, NULL, w + 1);
	
				// Print the authenticated prompt
				TCPPutROMString(MySocket, strAuthenticated);
				TCPFlush(MySocket);
				TelnetState = SM_AUTHENTICATED;
				// No break
			case SM_AUTHENTICATED:
				break;
		}
		// Save session state back into the static array
		hTelnetSockets[vTelnetSession] = MySocket;
		vTelnetStates[vTelnetSession] = TelnetState;
	}
}

BYTE GetTelnetConsole(void)
{

	BYTE		vTelnetSession,c;
	TCP_SOCKET	MySocket;

	for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
	{
		// Load up static state information for this session
		MySocket = hTelnetSockets[vTelnetSession];
		if (vTelnetStates[vTelnetSession] != SM_AUTHENTICATED) continue;
		if (TCPIsGetReady(MySocket)) 
		{
			TCPGet(MySocket, &c);
			return c;
		}
	}
	return 0;
}

BOOL PutTelnetConsole(char c)
{
	TCP_SOCKET	MySocket;
	WORD i;

	MySocket = hTelnetSockets[0];
	// Allow output in all states, but only when connected
	if (MySocket == INVALID_SOCKET) return 1;
	if (!TCPIsConnected(MySocket)) return 1;

	if (termbufidx < MAXTERMBUF)
	{
		termbuf[termbufidx++] = c;
		termbuftimer = 0;
		return 1;
	}
	// Determine output length (may be redacted pre-auth)
	BYTE tmpbuf[MAXTERMBUF + 8];
	WORD outlen = termbufidx;
	BOOL preauth = (vTelnetStates[0] != SM_AUTHENTICATED);
	if(preauth)
		outlen = redact_ips(termbuf, termbufidx, tmpbuf, sizeof(tmpbuf));

	if (TCPIsPutReady(MySocket) < outlen) 
	{
		StackTask();
		StackApplications();
		return 0;
	}
	if(preauth)
	{
		for(i = 0; i < outlen; i++) TCPPut(MySocket, tmpbuf[i]);
	}
	else
	{
		for(i = 0; i < termbufidx; i++) TCPPut(MySocket,termbuf[i]);
	}
	TCPFlush(MySocket);
	termbufidx = 0;
	termbuftimer = 0;
	return 0;
}

void ProcessTelnetTimer(void)
{
TCP_SOCKET	MySocket;
WORD i;

	MySocket = hTelnetSockets[0];
	// Allow output in all states, but only when connected
	if (MySocket == INVALID_SOCKET) return;
	if (!TCPIsConnected(MySocket)) return;

	if (termbufidx < 1) return;
	// Determine output length (may be redacted pre-auth)
	BYTE tmpbuf[MAXTERMBUF + 8];
	WORD outlen = termbufidx;
	BOOL preauth = (vTelnetStates[0] != SM_AUTHENTICATED);
	if(preauth)
		outlen = redact_ips(termbuf, termbufidx, tmpbuf, sizeof(tmpbuf));

	if (TCPIsPutReady(MySocket) < outlen) 
	{
		StackTask();
		StackApplications();
		return;
	}
	if(preauth)
	{
		for(i = 0; i < outlen; i++) TCPPut(MySocket, tmpbuf[i]);
	}
	else
	{
		for(i = 0; i < termbufidx; i++) TCPPut(MySocket,termbuf[i]);
	}
	termbufidx = 0;
	termbuftimer = 0;
	TCPFlush(MySocket);
	return;
}

#if 0

BOOL PutTelnetConsole(char c)
{

	BYTE		vTelnetSession,nconn;
	TCP_SOCKET	MySocket;


	nconn = 0;
	for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
	{
		if (vTelnetStates[vTelnetSession] == SM_AUTHENTICATED) nconn++;
	}
	nconn = 1;
	if (nconn > 0)
	{
		StackTask();
		StackApplications();
	}
	for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
	{
		// Load up static state information for this session
		MySocket = hTelnetSockets[vTelnetSession];
		if (vTelnetStates[vTelnetSession] != SM_AUTHENTICATED) continue;
		if (TCPIsPutReady(MySocket) < 1) 
		{
			StackTask();
			StackApplications();
			return 0;
		}
	}
	for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
	{
		// Load up static state information for this session
		MySocket = hTelnetSockets[vTelnetSession];
		if (vTelnetStates[vTelnetSession] != SM_AUTHENTICATED) continue;
		TCPPut(MySocket,c);
		TCPFlush(MySocket);
	}
	return 1;
}

#endif

void CloseTelnetConsole(void)
{
	BYTE		vTelnetSession;
	TCP_SOCKET	MySocket;

	termbufidx = 0;
	termbuftimer = 0;


	for(vTelnetSession = 0; vTelnetSession < MAX_TELNET_CONNECTIONS; vTelnetSession++)
	{
		// Load up static state information for this session
		MySocket = hTelnetSockets[vTelnetSession];
		if (MySocket == INVALID_SOCKET) continue;
		// Only authenticated sessions can invoke CloseTelnetConsole(), but be safe
		TCPDisconnect(MySocket);
		TelnetState = SM_PRINT_BANNER;
		vTelnetStates[vTelnetSession] = TelnetState;
	}
}


#endif	//#if defined(STACK_USE_TELNET_SERVER)
