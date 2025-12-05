# VOTER Firmware 4.00 – by Mason Nelson (N5LSN)
After a couple of years of operating a UHF simulcast VOTER system on firmware version 3.00, I have made several modifications and improvements to the firmware.

This document outlines the major functional improvements, behavior changes, and new capabilities introduced in Firmware 4.00.

> [!IMPORTANT]
> Several of these changes will conflict with existing VOTER documentation. Documentation updates will be required if this firmware becomes "official".

---

# Upgrading
Upgrading from any VOTER firmware version to 4.00 is directly compatible.

No settings will be changed and no data will be lost when upgrading.

---

# 1. Major Functional Improvements

## 1.1 DSPBEW Always Included
After several code optimizations, enough space has been freed to always compile DSPBEW mode functionality. Maintaining seperate firmware releases is no longer necessary.

## 1.2 Rebuilt and Simplified Menu System
The entire menu structure has been reorganized to make navigation more intuitive. Options are easier to find and grouped more logically, increasing legibility and ease of use.

#### VOTER Firmware Version 3.00:
```
1  - Serial # (1694) (which is MAC ADDR 00:04:A3:00:04:D2)
2  - VOTER Server Address (FQDN) (n0call.dyndns.net)
3  - VOTER Server Port (667),  4  - Local Port (Override) (0)
5  - Client Password (madcow1),  6  - Host Password (BLAH)
7  - Tx Buffer Length (640)
8  - GPS Data Protocol (0=NMEA, 1=TSIP) (1)
81 - GPS Type (0=Normal TSIP, 1=Trimble Thunderbolt) (0)
82 - GPS Time Offset (seconds to add for correction) (0)
9  - GPS Serial Polarity (0=Non-Inverted, 1=Inverted) (0)
10 - GPS PPS Polarity (0=Non-Inverted, 1=Inverted, 2=NONE) (1)
11 - GPS Baud Rate (9600)
12 - External CTCSS (0=Ignore, 1=Non-Inverted, 2=Inverted) (0)
13 - COR Type (0=Normal, 1=IGNORE COR, 2=No Receiver) (0)
14 - Debug Level (16)
15  - Alt. VOTER Server Address (FQDN) ()
16  - Alt. VOTER Server Port (Override) (0)
17  - DSP/BEW Mode NOT SUPPORTED
18 - "Duplex Mode 3" (0=DISABLED, 1-255 Hang Time) (1/10 secs) (0)
19 - Simulcast Launch Delay (0) (approx 200 ns, 5 = 1us, > 0 to ENA SC)
97 - RX Level,  98 - Status,  99 - Save Values to EEPROM
i - IP Parameters menu, o - Offline Mode Parameters menu, s - Squelch menu
q - Disconnect Remote Console Session, r - reboot system, d - diagnostics

Enter Selection (1-19,81-82,97-99,i,o,s,r,q,d) :
```
#### VOTER Firmware Version 4.00:
```
1  - Serial# (1234) [MAC=00:04:A3:00:04:D2]
2  - Client Pass (abcdefghijklmnopqr)
3  - Local Port (0)

4  - Server FQDN (n0call.dyndns.net)
5  - Server Port (1667)
6  - AltSvr FQDN ()
7  - AltSvr Port (0)
8  - Host Pass (abcdefghijklmnopqr)

9  - GPS Baud (9600)
10 - GPS SerPol [0=Norm,1=Inv] (1)
11 - PPS Pol [0=Norm,1=Inv,2=OFF] (0)
12 - GPS Proto [0=NMEA,1=TSIP] (0)
13 - GPS Type [0=Norm,1=Tbolt] (0)
14 - GPS TimeOfs [s] (0)

15 - ExtCTCSS [0=Ign,1=Norm,2=Inv] (1)
16 - COR [0=Norm,1=Ign,2=NoRX] (0)
17 - Duplex3 [0=OFF,1-255x0.1s] (0)
18 - TxBuf Len [x1/8000s] (3000)
19 - Launch Delay [x200ns,>0=ON] (0)

20 - Debug Opts (0)
21 - DSP/BEW (0)

97 - RX Level
98 - Status
99 - Save to EEPROM

i  - IP Menu
o  - Offline Menu
s  - Squelch Menu
a  - Auto Reboot Menu

q  - Disconnect
r  - Reboot

Sel:
```

## 1.3 Enhanced GPS Handling and Reliability
- The system now validates GPS data using the `$GPRMC` status field. Time sync is only accepted when GPS time is confirmed valid.
- GPS-related logs are clearer and more consistent, helping diagnose GPS lock issues more easily.

```
[<System Time Not Set>] GPS: Awaiting lock
[<System Time Not Set>] GPS: GPRMC 'A' valid
[<System Time Not Set>] GPS: Acq, sats=11
[2025/12/03 14:44:40.000] GPS: Sync OK
```

## 1.4 Immediate Telnet Console Access
- Telnet connects instantly and shows live system messages without requiring login.
- Login is still required to view or modify menus.
- IP addresses and hostnames are redacted when not logged in for better security.

```
VOTER #1234 - Version 4.00 (Compiled: Dec  3 2025)
<live console; press Enter to log in>

[<System Time Not Set>] GPS: Awaiting lock
[<System Time Not Set>] GPS: GPRMC 'A' valid
[<System Time Not Set>] GPS: Acq, sats=11
[2025/12/03 14:44:40.000] GPS: Sync OK
[2025/12/03 14:44:40.500] NET: Conn(Pri) [redacted]:1667
Logged in successfully...
```

This greatly improves real-time troubleshooting, especially during system startup.

## 1.5 Improved RX Level Display
The RX level bar graph has been replaced with a live numeric readout displaying both instantaneous and 15-second averaged deviation values.

The 15-second average helps compensate for natural fluctuation in the instantaneous measurement.

Deviation calculations have been tuned with an FM service monitor (previous firmware displayed roughly 300 Hz lower than actual).

#### VOTER Firmware Version 3.00
```
RX VOICE DISPLAY:                                                            
                                  v -- 3KHz        v -- 5KHz
|=============================>                                             |
```
#### VOTER Firmware Version 4.00:
```
RX Level:
Instant: 2.89 KHz | 15s Avg: 3.01 KHz
```

---

# 2. Connection and Status Behavior Enhancements

## 2.1 Offline Delay Configuration
Allows you to define how long the unit must remain offline before entering offline mode. This prevents quick network hiccups from triggering CWID messages or mode changes.

```
Offline Menu
...
12 - Offline Delay [s] (0)
```

## 2.2 Hardware Connection Status Output
- Introduces hardware-based connection status signaling on GPB7 for supported non-SMT boards.
- Adds manual override controls for the connection status output.

```
Offline Menu
...
13 - AuxOut [0=ConnStatus,1=Hi,2=Lo] (0)
```

This gives external equipment clear, reliable visibility of link state.

## 2.3 RCON Reporting
New information added to the status display (menu option 98) reads the dsPIC RCON register and shows the cause of the last system reset. This is useful for diagnosing unstable power, watchdog resets, crashes, or other unexpected shutdowns/reboots.

```
VOTER Client Status
...
= RCON =
Val=0x00C0
TRAP:N IO:N CM:N
EXTR:Y SW:Y WD:N
SLEP:N IDL:N BOR:N POR:N
```
- **TRAPR**: Trap conflict
- **IOPWR**: Illegal opcode/uninit
- **CM**: Config mismatch
- **EXTR**: MCLR reset
- **SWR**: RESET instruction
- **WDTO**: Watchdog timeout
- **SLEEP**: PWRSAV #SLEEP
- **IDLE**: PWRSAV #IDLE
- **BOR**: Brown-out reset
- **POR**: Power-on reset
---

# 3. New Automation and Reliability Features

## 3.1 Scheduled Reboots
You can now configure automatic reboots at scheduled times. This helps long-running installations maintain consistent operation.

```
Auto Reboot Menu - UTC: 2025/12/05 04:31:35.540

1 - Sched [0=Off,1=Daily,2=Wkly] (0)
2 - Day [0=Sun...6=Sat] (0)
3 - Hour (0)
4 - Min (0)
```

## 3.2 Cold Power Reboot Timer
After a cold power-up (first boot after power loss), the system can automatically reboot after a user-specified interval. This helps re-sync to GPS modules that report incorrect UTC time for some time after starting up (leap seconds, etc).

```
Auto Reboot Menu - UTC: 2025/12/05 04:31:35.540
...
5 - Cold Power Reboot [m] (0=Off) (0)
```

## 3.3 GPS Reset Output
Supported hardware can trigger a reset of the onboard GPS module vie GPB6 whenever the dsPIC bootloader starts. This enables an onboard GPS module to be reset remotely along with the rest of the system.

## 3.4 PTT Inhibit During Connect/Disconnect
A brief, 100ms PTT inhibit timer prevents keying the transmitter too early when handling connections/disconnections to/from the host. This protects systems that switch external configurations based on connection status and avoids collisions and missed PTT events.

---

# 4. Default Behavior Changes

These defaults are chosen to match modern setups and reduce required initial configuration steps.

- UDP port changed from 667 to 1667 to meet modern Linux requirements and match ASL3’s updated defaults.
- GPS serial baud rate increased from 4800 to 9600, which is more common.
- PPS (non-inverted) is now enabled by default, which is more common.

These changes make initial deployment faster in most environments.

---

# 5. Logging System Improvements

A rebuilt logging infrastructure provides clearer and more consistent output while reducing code size. Highlights:

- Unified formatting and reusable messages.
- Improved tracking of system states.
- Better visibility into GPS, connection transitions, and system behavior.

```
VOTER #1234 - Version 4.00 (Compiled: Dec  3 2025)
<live console; press Enter to log in>

[<System Time Not Set>] GPS: GPRMC 'A' valid
[<System Time Not Set>] GPS: Awaiting lock
[<System Time Not Set>] GPS: Acq, sats=11
[<System Time Not Set>] GPS: Sync OK
[2025/12/03 15:14:03.500] NET: Conn(Pri) [redacted]:1667
[2025/12/03 15:14:23.700] ERR: Host TO
[2025/12/03 15:14:23.700] NET: Disc(Pri) [redacted]:1667
[2025/12/03 15:14:26.540] WARN: DNS fail [redacted]
[2025/12/03 15:14:38.300] GPS: GPRMC 'V' void
[2025/12/03 15:14:38.360] GPS: Lost, restart
[2025/12/03 15:14:38.360] GPS: Awaiting lock
[2025/12/03 15:14:38.360] GPS: GPRMC 'A' valid
[2025/12/03 15:14:38.360] GPS: Acq, sats=11
[2025/12/03 15:14:42.980] GPS: Sync OK
[2025/12/03 15:14:42.980] WARN: GPS PPS TO
[2025/12/03 15:14:42.980] GPS: Sync lost
[2025/12/03 15:14:47.980] GPS: Sync OK
[2025/12/03 15:15:01.020] NET: Conn(Pri) [redacted]:1667
```

## 5.1 Added RX/TX Event Debug Output (DebugLevel bit 1)
RX/TX event logging allows visibility into radio events and system inputs/outputs. Useful for troubleshooting external connected radio systems.

```
Debug Options

1  - RX/TX log
2  - Stats
16 - NoTOS
32 - GPS
64 - FixGPS

Note: Bitmask; Sum values (e.g. 3=1+2)

Enter new value: 1
Changed

[2025/12/03 15:17:30.960] RX: COR det, RSSI=255
[2025/12/03 15:17:31.140] RX: CTCSS det
[2025/12/03 15:17:31.400] TX: PTT on buf=375ms tim=-355ms
[2025/12/03 15:17:36.820] RX: COR lost, RSSI=0
[2025/12/03 15:17:36.860] RX: CTCSS lost
[2025/12/03 15:17:47.520] TX: PTT off
```

- COR detected/lost
- CTCSS detected/lost
- PTT asserted/de-asserted
  - `buffer=` shows the configured transmit buffer length in milliseconds
  - `timing=` (when GPS/PPS enabled) shows timing relative to playback deadline in milliseconds
    - Calculated as: (time since last RX packet) - (buffer length - 1 frame)
    - **Negative values** (e.g., -355ms) indicate audio arrived early with good buffer headroom
    - **Small positive values** (0-60ms) indicate audio is close to playback deadline (PTT activates when ≤ 60ms)
    - **Values >60ms** would cause PTT to drop (audio missed deadline by more than tolerance window)
    - The 60ms threshold provides hysteresis to handle normal network jitter without PTT flutter
    - In a properly functioning simulcast system, expect negative values showing adequate buffer margin
    - This is critical for verifying timing synchronization in simulcast configurations

## 5.2 Added STAT Debug Output (DebugLevel bit 2)
A new compact, once-per-second status emission for both TX and RX paths. Useful for diagnosing latency, packet behavior, RSSI, buffer utilization, and overall system performance.

```
Debug Options

1  - RX/TX log
2  - Stats
16 - NoTOS
32 - GPS
64 - FixGPS

Note: Bitmask; Sum values (e.g. 3=1+2)

Enter new value: 3
Changed

[2025/12/03 15:19:18.820] RX: COR det, RSSI=255
[2025/12/03 15:19:19.000] RX: CTCSS det
[2025/12/03 15:19:19.280] TX: PTT on buf=375ms tim=-355ms
[2025/12/03 15:19:19.560] TX STAT: tx=12956 host=0 q=337ms drain=410 miss=0 opt=0x0A
[2025/12/03 15:19:19.560] RX STAT: rssi=63% samples=234 last_samplecnt=8000 missed=0 ms=0 inx=2680 inb=1
[2025/12/03 15:19:20.560] TX STAT: tx=13006 host=0 q=87ms drain=2410 miss=0 opt=0x0A
[2025/12/03 15:19:20.560] RX STAT: rssi=100% samples=228 last_samplecnt=8000 missed=0 ms=0 inx=2680 inb=1
[2025/12/03 15:19:21.560] TX STAT: tx=13056 host=0 q=212ms drain=1410 miss=0 opt=0x0A
[2025/12/03 15:19:21.560] RX STAT: rssi=100% samples=228 last_samplecnt=8000 missed=0 ms=0 inx=2680 inb=1
[2025/12/03 15:19:22.560] TX STAT: tx=13106 host=0 q=337ms drain=411 miss=0 opt=0x0A
[2025/12/03 15:19:22.560] RX STAT: rssi=100% samples=229 last_samplecnt=8000 missed=0 ms=0 inx=2680 inb=1
[2025/12/03 15:19:23.560] TX STAT: tx=13156 host=0 q=87ms drain=2413 miss=0 opt=0x0A
[2025/12/03 15:19:23.560] RX STAT: rssi=100% samples=228 last_samplecnt=8000 missed=0 ms=0 inx=2680 inb=1
[2025/12/03 15:19:24.260] RX: COR lost, RSSI=0
[2025/12/03 15:19:24.280] RX: CTCSS lost
[2025/12/03 15:19:24.560] TX STAT: tx=13206 host=0 q=212ms drain=1413 miss=0 opt=0x0A
[2025/12/03 15:19:25.560] TX STAT: tx=13256 host=0 q=337ms drain=415 miss=0 opt=0x0A
[2025/12/03 15:19:26.560] TX STAT: tx=13306 host=0 q=87ms drain=2415 miss=0 opt=0x0A
[2025/12/03 15:19:27.560] TX STAT: tx=13356 host=0 q=212ms drain=1418 miss=0 opt=0x0A
[2025/12/03 15:19:28.560] TX STAT: tx=13406 host=0 q=337ms drain=419 miss=0 opt=0x0A
[2025/12/03 15:19:29.560] TX STAT: tx=13456 host=0 q=87ms drain=2419 miss=0 opt=0x0A
[2025/12/03 15:19:29.700] TX: PTT off
```

### TX STAT Fields

**Format:** `TX STAT: tx=<txseqno> host=<host_txseqno> q=<queued_ms> drain=<txdrainindex> miss=<missed> opt=<option_flags>`

- **tx=** Client transmit sequence number
  - Increments for each audio frame transmitted locally

- **host=** Host transmit sequence number
  - Sequence number from the VOTER host server for received audio packets
  - Value of 0 indicates no audio packets have been received from the host yet
  - Increments as the host sends audio frames to this client

- **q=** Queued audio buffer depth in milliseconds
  - Shows how much audio is currently buffered and waiting to be transmitted
  - Calculated from the difference between fill index and drain index in the circular buffer

- **drain=** Transmit buffer drain index (in samples)
  - Current position in the circular transmit buffer where audio is being read for TX
  - Wraps around when reaching the configured `TxBufferLength`

- **miss=** Missed packet counter
  - Total number of audio packets that arrived too late or couldn't be placed in the buffer
  - In GPS/PPS mode: packets that arrived outside the valid timing window
  - In non-GPS mode: packets with sequence numbers that couldn't fit in the buffer

- **opt=** Option flags (hexadecimal)
  - Bitmask of active options negotiated with the host
  - `0x01` = Flat audio (no de-emphasis)
  - `0x02` = Send audio always (master mode)
  - `0x04` = No CTCSS filtering
  - `0x08` = Master timing source (no packet delay)
  - `0x10` = ADPCM codec (vs. μ-law)
  - `0x20` = Mix mode enabled
  - Example: `0x0A` = bits 1 and 3 set (send always + master timing)

**TX STAT is only printed when PTT is asserted**, as there's no meaningful transmit activity to report otherwise.

### RX STAT Fields

**Format:** `RX STAT: rssi=<rssi_pct>% samples=<rssi_count_sec> last_samplecnt=<last_samplecnt> missed=<missed> ms=<ms_since_rx> inx=<last_rxpacket_index> inb=<last_rxpacket_inbounds>`

- **rssi=** Received Signal Strength Indicator (percentage)
  - Average RSSI over the last 1 second interval, converted to percentage (0-100%)
  - Raw RSSI values (0-255) are averaged and scaled
  - Averaged only over actual samples received during the interval

- **samples=** Number of RSSI samples collected in the last second
  - Count of individual RSSI measurements taken during the 1-second reporting interval
  - Used to calculate the RSSI average

- **last_samplecnt=** ADC sample count for audio level monitoring
  - Running count of audio ADC samples processed (typically 8000 samples/second)

- **missed=** Same as TX missed packet counter
  - Total cumulative count of packets that couldn't be properly buffered
  - Shared between TX and RX statistics as it's a system-wide metric
  - See TX STAT description for details

- **ms=** Milliseconds since last received audio packet from host
  - Time elapsed since the most recent audio packet was received from the VOTER host
  - Value of 0 indicates a packet was just received

- **inx=** Last received packet buffer index (in samples)
  - Calculated buffer position where the most recent host audio packet should be placed
  - In GPS/PPS mode: based on packet timestamp vs. current system time, converted to sample count
  - In non-GPS mode: based on host sequence number with buffer-size-dependent offsets
  - Valid range: 0 to (TxBufferLength - 320 samples)

- **inb=** Last packet in-bounds flag (0 or 1)
  - **1** = Last received packet was within valid buffer timing window and was used
  - **0** = Last received packet was outside timing window and was rejected

**RX STAT is only printed when COR is active and CTCSS is valid** (if CTCSS is configured), as there's no meaningful reception data otherwise.

---

# 6. Optimizations and Code Cleanups

These changes primarily help fit new features while keeping performance stable. They do not meaningfully change user workflows.

- Replaced CRC-32 lookup table with a compact calculation method.
- Removed the (unused?) diagnostic menu.
- Removed GGPS and POCSAG features that were not used.
- Numerous string optimizations, typo fixes, and code cleanups.

---

# 7. Additional Quality-of-Life Improvements

- Password entry is now hidden from the console after login.