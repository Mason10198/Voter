# DSP/BEW Feature: Baseband Examination Window

> Disclaimer: This document was generated with AI assistance to improve the available documentation for the DSP/BEW feature. It is based on repository documentation, source comments, source code, and implementation analysis. Technical details should be validated against the current source before being used for firmware design, modification, or reimplementation.

This document captures the documentation, source comments, code paths, and inferred behavior for the DSP/BEW feature in the VOTER/RTCM firmware. The repository consistently calls the feature `DSPBEW`, `DSP/BEW`, or `BEW`.

The goal is to preserve enough detail that the feature can be recreated or reimplemented without relying on the original firmware.

## Executive summary

DSP/BEW means "Digital Signal Processor / Baseband Examination Window." It is an RSSI protection feature for receivers whose discriminator noise output is a good signal-strength indicator when there is no receiver audio modulation, but becomes unreliable when the received signal contains strong high-frequency voice or high-deviation audio.

The normal VOTER/RTCM signal-strength method reads a receiver noise-voltage ADC channel, smooths it, converts it to an 8-bit RSSI value, and sends that RSSI in each audio packet. That works well for receivers whose discriminator output contains high-frequency noise that falls as carrier strength improves. Some receivers, explicitly called out in the existing documentation as Motorola Quantar-class receivers, do not provide enough clean high-frequency noise spectrum under strong modulation. Their "noise" measurement can be contaminated by baseband audio, causing the voter server to see incorrect signal strength while the receiver is actively passing loud/high-frequency audio.

DSP/BEW does not replace the receiver, ADC, squelch, audio encoding, or packet protocol. It adds a short spectral test over a 32-sample baseband audio window. If that window contains enough baseband audio energy to make the noise/RSSI measurement suspect, the firmware temporarily holds the last known-good RSSI/noise value. When the baseband audio energy drops back below the threshold, normal RSSI tracking resumes.

In short:

```text
Normal method:
  noise ADC -> smoothed noise -> RSSI -> UDP audio packet

DSP/BEW method:
  audio window -> FFT -> baseband energy test
  if audio energy is low:  noise ADC -> smoothed noise -> RSSI -> UDP packet
  if audio energy is high: hold previous valid noise/RSSI -> UDP packet
```

## Source inventory

The relevant repository material is:

| Area | File or commit | What it contains |
| --- | --- | --- |
| Existing prose docs | `VOTER_RTCM-firmware/README.md`, section "Baseband Examination Window (BEW) Firmware" | Explains the receiver problem, Quantar-type use case, RSSI hold behavior, and warning that older DSPBEW builds displaced diagnostics. |
| 4.00 release note | `VOTER_RTCM-firmware/firmware-images/4.00/VOTER_Firmware_4.00.md`, section "DSPBEW Always Included" | Says firmware 4.00 always compiles DSPBEW after ROM optimizations, removing the need for separate DSPBEW firmware images. |
| Current implementation | `VOTER_RTCM-firmware/build-files/src/Voter.c` | Compile-time feature define, FFT constants, audio window extraction, DSP calls, RSSI hold logic, menu option, startup twiddle-factor init. |
| Config field | `VOTER_RTCM-firmware/build-files/src/TCPIP Stack/StackTsk.h` | `AppConfig.BEWMode`, stored in persistent application config. |
| DSP support library | `VOTER_RTCM-firmware/build-files/src/DSP Library/fft.s`, `bitrev.s`, `cplxsqrmag.s`, `inittwid.c`, `flt2frct.c`, `dspcommon.inc` | Microchip dsPIC DSP routines used by the BEW FFT path. |
| Project files | `VOTER_RTCM-firmware/build-files/voter.mcp`, `voter-smt.mcp` | Include the DSP library source files in both through-hole VOTER and SMT RTCM builds. |
| Original feature commit | `80e16bb` | Adds DSP/BEW feature, FFT constants, `rssiheld`, `lastvnoise32`, `fftresult`, and `BEWMode`. |
| Conditional/separate build support | `a5f6653` | Adds better DSPBEW support, separate DSPBEW project files, `#ifdef DSPBEW`, `ROMNOBEW`, and BEW mode 2 input scaling. |
| Original README text | `7587764` | Adds the original README-DSPBEW text that was later folded into the current README. |
| History-slot bug fix | `d6aa299` | Ensures all three `lastvnoise32[]` history slots are initialized when COR first qualifies. |
| Master timing bug fix | `e645d97` | Moves the FFT work inside the "filled packet is ready to send" condition, fixing a DSPBEW/master timing issue. |
| Current 4.00 behavior | `a330bf3`, `9337344`, `83595c2` | ROM savings make DSPBEW fit, then DSPBEW is forced on by default, then refactored as an optional feature define that is currently enabled. |

## Current code map

Line numbers are for the current checked-out source when this document was written.

| File | Lines | Relevance |
| --- | ---: | --- |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 98-99 | Current `DSPBEW` compile-time enable. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 169-173 | DSPBEW build reduces `MAX_BUFLEN` from 6400 to 4800. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 222-244 | FFT constants, DSP buffers, twiddle-factor storage, `ROMNOBEW`. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 402-403 | `rssi` and `rssiheld` state. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 422-423 | `vnoise32` and `lastvnoise32[]` state. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 811-828 | `calcrssi()` conversion from noise ADC value to 0-255 RSSI. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 1073-1078 | ADC ISR comments documenting 62.5 us ADC cadence and 8 kHz audio cadence. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 1158-1342 | Audio sample conversion and double-buffer fill path. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 2864-3000 | Main DSP/BEW FFT detector and outgoing `rssiheld` packet behavior. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 3595-3656 | Secondary-loop squelch service, `qualnoise` gate, `vnoise32` update, RSSI hold. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 5236-5240 | Menu display for DSP/BEW option. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 5545-5549 | Startup twiddle-factor initialization. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 5901-5905 | Menu setter accepting BEW modes 0, 1, and 2. |
| `VOTER_RTCM-firmware/build-files/src/Voter.c` | 6323-6385 | Config defaults; `AppConfig` is zeroed, so default `BEWMode` is 0. |
| `VOTER_RTCM-firmware/build-files/src/squelch.c` | 122-307 | Normal raw-noise squelch state machine that BEW does not replace. |
| `VOTER_RTCM-firmware/build-files/src/TCPIP Stack/StackTsk.h` | 160 | Persistent `AppConfig.BEWMode` field. |
| `VOTER_RTCM-firmware/build-files/src/DSP Library/fft.s` | 28-38 | DSP FFT assumptions: input magnitude, 1/N scaling, output bit reversal. |
| `VOTER_RTCM-firmware/build-files/src/DSP Library/cplxsqrmag.s` | 38-60 | Squared magnitude operation and in-place source/destination allowance. |
| `VOTER_RTCM-firmware/build-files/src/DSP Library/inittwid.c` | 63-91 | Twiddle-factor initialization API and conjugate flag. |

## Existing documentation meaning

The README describes the problem this feature solves:

1. FM receiver discriminator output normally contains sub-audible content, normal audio, and higher-frequency noise.
2. The higher-frequency content can be used as a signal-quality proxy because strong carrier suppresses noise.
3. Some receivers do not provide enough usable high-frequency noise content under modulation.
4. They may still provide valid no-modulation noise, but strong high-frequency/high-deviation audio can interfere with signal-strength analysis.
5. DSP/BEW examines a baseband audio "window" and decides when the RSSI/noise value should be held.

The old README warning says DSPBEW builds lost the Diagnostics Menu due dsPIC code-space limits. That was true for the older split firmware era. In current firmware 4.00, the source has `#define DSPBEW`, and the 4.00 release notes say DSPBEW is always included after code-size optimizations.

## Build and configuration behavior

### Compile-time switch

Current `Voter.c` defines:

```c
// Optional features - comment out to remove from build
#define DSPBEW
```

When `DSPBEW` is defined:

1. The DSP library interface is included through `<dsp.h>`.
2. FFT constants and buffers are compiled.
3. The transmit buffer maximum is reduced from 6400 samples to 4800 samples.
4. Menu option 21 displays and can change `AppConfig.BEWMode`.
5. Startup initializes FFT twiddle factors.
6. The RSSI path uses BEW-aware history and hold logic.

When `DSPBEW` is not defined:

1. The menu prints `DSP/BEW [N/A]`.
2. Option 21 is rejected.
3. `rssiheld` is assigned directly from `rssi`.
4. The RSSI calculation uses current `vnoise32` instead of BEW history.
5. The maximum transmit buffer returns to 6400 samples.

### Runtime menu

Current firmware exposes BEW at main menu item 21:

```text
21 - DSP/BEW (%d)
```

The setter accepts values 0, 1, or 2:

```c
case 21: // BEW Mode
    if ((sscanf(cmdstr,"%u",&i1) == 1) && (i1 <= 2))
    {
        AppConfig.BEWMode = i1;
        ok = 1;
    }
    break;
```

The config struct currently declares this as `BOOL BEWMode`, but the code intentionally stores and tests values 0, 1, and 2. Treat it as a small integer mode field, not a strict boolean, when reimplementing.

### Mode values

| `BEWMode` | Meaning in code | Practical behavior |
| --- | --- | --- |
| 0 | Disabled | `qualnoise` is forced true. RSSI is not held due FFT contamination. In a compiled-DSPBEW build, some BEW history plumbing still exists, but the contamination gate is disabled. |
| 1 | Enabled, normal scaling | Uses the 32-sample FFT test. Converts u-law samples to linear and divides by 2 before placing them into Q15 fractional FFT input. |
| 2 | Enabled, higher sensitivity/full input scaling | Uses the same FFT test, but clips the linear sample to +/-16383 and does not divide by 2. For non-clipped samples this roughly doubles FFT input amplitude and quadruples squared-magnitude energy. |

Default config is effectively 0 because `InitAppConfig()` zeroes the entire `AppConfig` struct and does not explicitly assign `BEWMode`.

## Relevant constants

Current constants:

```c
#define FFT_BLOCK_LENGTH        32
#define LOG2_BLOCK_LENGTH       5
#define FFT_TOP_SAMPLE_BUCKET   16 // 3000 Hz
#define FFT_MAX_RESULT          10
#define QUALCOUNT               4
```

Other relevant constants:

```c
#define FRAME_SIZE              160
#define MASTER_TIMING_DELAY     50
#define ADCOTHERS               3
#define ADCSQNOISE              0
#define ADCDIODE                2
#define ADCSQPOT                1
```

Timing derived from comments and code:

| Quantity | Value | Source/derivation |
| --- | ---: | --- |
| ADC interrupt period | 62.5 us | Timer 3 triggers ADC every 62.5 us. |
| RX audio sample rate | 8000 samples/s | The ADC ISR alternates RX audio and other ADC channels; RX audio is processed every other interrupt. |
| Audio frame size | 160 samples | `FRAME_SIZE`; one UDP audio payload at u-law rate. |
| Audio frame duration | 20 ms | 160 / 8000. |
| BEW FFT window size | 32 samples | `FFT_BLOCK_LENGTH`. |
| BEW time window | 4 ms | 32 / 8000. |
| FFT bin spacing | 250 Hz | 8000 / 32. |
| Squelch service cadence | about 4.125 ms | `sqlcount >= 33`; 33 / 8000. |
| `QUALCOUNT` post-detection hold | about 16.5 ms after the last bad service pass | 4 service passes * 4.125 ms, due `QUALCOUNT = 4`. |

Important comment mismatch: with a 32-point FFT at 8 kHz, bin spacing is 250 Hz. The loop sums bins 2 through 15, which correspond to about 500 Hz through 3750 Hz. Source comments say `FFT_TOP_SAMPLE_BUCKET 16 // 3000 Hz` and "below 2000 Hz"; those comments do not match the current constants and sample rate. A faithful reimplementation should preserve the code behavior, not the stale comments, unless the threshold is recalibrated.

## Normal RSSI and squelch method

The non-BEW method relies on the receiver noise-voltage ADC channel.

### ADC scheduling

Timer 3 triggers ADC conversion every 62.5 us. The ADC ISR alternates between:

1. RX audio on AN0.
2. One of the "other" ADC channels: noise/RSSI, squelch pot, diode voltage.

The "other" readings are stored as 10-bit values:

```c
adcothers[adcindex++] = index >> 2;
```

`ADCSQNOISE` is the noise/RSSI input. In the original through-hole VOTER board comments, AN2 is "Noise Voltage (RSSI)"; SMT uses a different pin map but the logical channel is the same.

### Squelch state machine

`service_squelch()` in `squelch.c` uses raw `adcothers[ADCSQNOISE]` for COR/squelch decisions. It:

1. Stores noise samples in a 16-entry ring buffer.
2. Temperature-compensates the squelch threshold using the diode calibration value.
3. Builds a hysteresis band:
   - `sqposm = sqposcomp - AppConfig.Hysteresis`
   - `sqposp = sqposcomp + AppConfig.Hysteresis`
4. Maintains a 31/32 modified average:
   - `mavnoise32 = ((mavnoise32 * 31) + noise) >> 5`
5. Opens squelch when average noise is lower/quieter than the lower threshold.
6. Closes squelch when average noise is higher/noisier than the upper threshold.
7. Uses a "noise 50 ms ago" check to close quickly on unkey after a strong signal.

DSP/BEW does not replace this squelch state machine. The raw squelch service still runs before BEW RSSI gating. BEW only changes the separate smoothed noise/RSSI value used for packet RSSI and status.

### Normal packet RSSI path

The packet RSSI path uses `vnoise32`, a separate smoothed noise estimate from the same ADC noise channel. In current source the update is:

```c
vnoise32 = ((vnoise32 * 3) + ((DWORD)adcothers[ADCSQNOISE] << 3)) >> 2;
```

The `<< 3` stores the 10-bit ADC value with three fractional bits for smoothing. When converting to RSSI, the code shifts back down:

```c
rssi = calcrssi(mynoise >> 3);
```

Without DSPBEW, `mynoise = vnoise32`, and the value sent in the audio packet is just:

```c
rssiheld = rssi;
```

### RSSI conversion

`calcrssi(WORD val)` maps lower noise voltage to higher signal strength.

For `val < 200`, the "Chuck RSSI" part is linear:

```c
x = 255 - val;
```

For `val >= 200`, the older logarithmic calculation is used:

```c
d = (((DWORD)val + 1) << 8);
i = log2fix(isqrt(d) << 4);
x = 255 - ((i << 3) / 39);
if (x < 0) x = 0;
```

So RSSI is an 8-bit value from 0 to 255, where larger means stronger/quieter. If a qualified carrier exists but RSSI would calculate below 1, the code forces it to 1 so a valid received signal is not transmitted as zero RSSI. If `SqlNoiseGain` is zero, RSSI is forced to zero.

## DSP/BEW algorithm in detail

DSP/BEW adds two cooperating gates:

1. A per-audio-frame FFT test in `process_udp()`.
2. A faster secondary-loop RSSI update gate in `secondary_processing_loop()`.

The two gates share the global `fftresult`, `vnoise32`, `lastvnoise32[]`, and `rssiheld` state.

### Data structures

When `DSPBEW` is enabled:

```c
fractcomplex sigCmpx[FFT_BLOCK_LENGTH]
fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2]
DWORD fftresult
DWORD vnoise32
DWORD lastvnoise32[3]
BYTE rssi
BYTE rssiheld
```

`sigCmpx` is placed in dsPIC Y memory and aligned for the DSP library. `twiddleFactors` is placed in X memory unless `FFTTWIDCOEFFS_IN_PROGMEM` is used.

Startup initializes twiddle factors once:

```c
TwidFactorInit(LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);
```

The Microchip FFT routine expects Q15 fractional complex input with real and imaginary magnitudes less than 0.5. It scales internally by 1/2 per stage, so a 32-point FFT output is scaled by 1/32.

### When the FFT runs

The FFT runs only when a completed audio buffer is ready to process:

```c
if (filled && ((fillindex > MASTER_TIMING_DELAY) ||
    (option_flags & OPTION_FLAG_MASTERTIMING)))
{
    // DSP/BEW FFT and packet transmit path
}
```

This placement matters. A historical commit explicitly moved the FFT inside the "filled packet is ready" condition to fix a DSPBEW/master timing problem. Reimplementations should tie the FFT result to the completed audio frame being considered for transmission.

### Window selection

The completed audio buffer is the one not currently being filled:

```c
audio_buf[filling_buffer ^ 1]
```

DSP/BEW examines the first 32 encoded samples/bytes in that completed buffer:

```c
for (i = 0; i < FFT_BLOCK_LENGTH; i++)
{
    x = ulawtabletx[audio_buf[filling_buffer ^ 1][i]];
    ...
}
```

At the normal u-law sample rate this is a 4 ms window at the beginning of the 20 ms audio packet.

Caveat: this code always uses `ulawtabletx[]` to convert the audio byte to signed linear audio. If the host has requested ADPCM (`OPTION_FLAG_ADPCM`), the same buffer contains packed ADPCM data, not u-law bytes. The source does not special-case ADPCM before running the BEW FFT. The normal documented/default mode is u-law, and the BEW implementation appears designed around u-law frames.

### Input scaling

For mode 1:

```c
sigCmpx[i].real = x / 2;
sigCmpx[i].imag = 0x0000;
```

For mode 2:

```c
if (x > 16383) x = 16383;
if (x < -16383) x = -16383;
sigCmpx[i].real = x;
sigCmpx[i].imag = 0x0000;
```

This is best understood as Q15 fractional preparation:

1. Mode 1 halves the u-law table output to keep it near the DSP library's required magnitude.
2. Mode 2 clips first, then uses the unclipped lower-amplitude values directly. This makes lower-level audio produce larger FFT magnitudes than mode 1, but prevents violating the +/-0.5 fractional limit.

Because the next step squares magnitudes, doubling the input amplitude increases the energy metric by roughly 4x until clipping.

### FFT and magnitude calculation

The DSP operations are:

```c
FFTComplexIP(LOG2_BLOCK_LENGTH, &sigCmpx[0], &twiddleFactors[0], COEFFS_IN_DATA);
BitReverseComplex(LOG2_BLOCK_LENGTH, &sigCmpx[0]);
SquareMagnitudeCplx(FFT_BLOCK_LENGTH, &sigCmpx[0], &sigCmpx[0].real);
```

The Microchip FFT implementation:

1. Is an in-place complex decimation-in-frequency FFT.
2. Expects natural-order input.
3. Produces bit-reversed output.
4. Scales each stage by 1/2, so total output is scaled by 1/N.

`BitReverseComplex()` restores natural bin order.

`SquareMagnitudeCplx()` writes:

```text
mag[k] = real(FFT[k])^2 + imag(FFT[k])^2
```

The destination is `&sigCmpx[0].real`. Since a `fractcomplex` array is contiguous 16-bit real/imag pairs, writing 32 real magnitude values at that address reuses the same memory. The code then treats `sigCmpx` as a contiguous `unsigned int` magnitude array:

```c
wp = (unsigned int *)&sigCmpx[0];
```

### Energy summation

The code computes:

```c
fftresult = 0;
for (i = 0; i < FFT_TOP_SAMPLE_BUCKET; i++)
{
    if (i >= 2) fftresult += *wp;
    wp++;
}
```

With current constants, this means:

```text
fftresult = sum(mag[2], mag[3], ..., mag[15])
```

At 8 kHz and 32 FFT points:

| Bin | Frequency | Used |
| ---: | ---: | --- |
| 0 | 0 Hz/DC | no |
| 1 | 250 Hz | no |
| 2 | 500 Hz | yes |
| 3 | 750 Hz | yes |
| 4 | 1000 Hz | yes |
| 5 | 1250 Hz | yes |
| 6 | 1500 Hz | yes |
| 7 | 1750 Hz | yes |
| 8 | 2000 Hz | yes |
| 9 | 2250 Hz | yes |
| 10 | 2500 Hz | yes |
| 11 | 2750 Hz | yes |
| 12 | 3000 Hz | yes |
| 13 | 3250 Hz | yes |
| 14 | 3500 Hz | yes |
| 15 | 3750 Hz | yes |
| 16 | 4000 Hz/Nyquist | no |

The intent is to ignore DC, low-frequency tone/squelch-related energy, and CTCSS/sub-audible content, then measure enough normal speech/baseband energy to know whether the RSSI noise measurement is being contaminated.

### Quality decision

The primary decision is:

```c
qualnoise = ((fftresult <= FFT_MAX_RESULT));
```

With current constants:

```text
qualnoise = fftresult <= 10
```

The name is confusing. Here, `qualnoise == true` means "the noise/RSSI sample is qualified/usable." It does not mean "there is noise." In fact, low FFT energy means there is not enough baseband audio energy to corrupt the noise measurement.

Then:

```c
if (!AppConfig.BEWMode) qualnoise = 1;
```

So mode 0 disables the BEW gate by forcing all noise/RSSI samples to be considered usable.

### What happens when the FFT says the RSSI is contaminated

When `qualnoise` is false in `process_udp()`:

```c
vnoise32 = lastvnoise32[2] = lastvnoise32[1] = lastvnoise32[0];
rssiheld = calcrssi(vnoise32 >> 3);
```

This does three things:

1. Restores `vnoise32` to the oldest valid value in the three-entry history.
2. Collapses all three history entries to the same held value.
3. Recomputes `rssiheld` from that held noise.

The effect is a hard freeze of the RSSI/noise baseline during high baseband energy.

At the end of `process_udp()`, history advances only if `qualnoise` is true:

```c
if (qualnoise)
{
    lastvnoise32[0] = lastvnoise32[1];
    lastvnoise32[1] = lastvnoise32[2];
    lastvnoise32[2] = vnoise32;
}
```

So contaminated windows do not enter the valid-noise history.

### Secondary-loop gating and hold extension

`secondary_processing_loop()` runs the RSSI update gate whenever `sqlcount >= 33`. It first services the raw squelch state machine, then computes:

```c
qualcor = (HasCOR() && HasCTCSS());
qualnoise = ((fftresult <= FFT_MAX_RESULT) || (!qualcor));
if (!AppConfig.BEWMode) qualnoise = 1;
```

This is similar to the transmit path but with one addition: if there is no qualified carrier/CTCSS (`!qualcor`), noise is considered usable. That prevents BEW from freezing noise while idle or while no usable receiver signal is active.

The debounce/hold extension is:

```c
if (!qualnoise) qualcnt = 0;

if (qualcnt <= QUALCOUNT)
{
    qualcnt++;
    qualnoise = 0;
}
```

After a contaminated result, `qualcnt` keeps `qualnoise` false for a few extra squelch-service passes even after the FFT result becomes good again. With `QUALCOUNT = 4` and a roughly 4.125 ms service interval, this adds about 16.5 ms of hold after the last bad pass. This avoids rapid chatter around the threshold.

### RSSI smoothing under BEW

The normal smoothed noise estimate only updates when `qualnoise` is true:

```c
if (sql2)
{
    if (qualcor && (!wascor))
    {
        lastvnoise32[0] = lastvnoise32[1] = lastvnoise32[2] =
            vnoise32 = (DWORD)adcothers[ADCSQNOISE] << 3;
    }
    else
    {
        if (qualnoise)
            vnoise32 = ((vnoise32 * 3) +
                ((DWORD)adcothers[ADCSQNOISE] << 3)) >> 2;
    }
    wascor = qualcor;
}
```

Details:

1. `sql2` toggles each service pass, so this update occurs every other `sqlcount >= 33` pass.
2. On the first qualified carrier after no carrier, all valid-noise history slots are initialized to the current raw noise value. This was added by a historical bug fix so the hold buffer is not stale at the start of a receive.
3. During contaminated baseband audio (`qualnoise == false`), `vnoise32` does not incorporate the current ADC noise sample.
4. During usable windows (`qualnoise == true`), `vnoise32` follows the current ADC noise with a 3/4 old, 1/4 new IIR average.

Then the code chooses what noise value to convert to RSSI:

```c
#ifdef DSPBEW
    if (qualnoise) mynoise = (WORD)lastvnoise32[1];
#else
    mynoise = vnoise32;
#endif

rssi = calcrssi(mynoise >> 3);
```

In a DSPBEW build, `mynoise` is static. If `qualnoise` is false, `mynoise` is not reassigned, so RSSI remains based on the prior accepted value. If `qualnoise` is true, it uses the middle entry of the three-entry valid history (`lastvnoise32[1]`) rather than the immediate `vnoise32`. This adds a small delay and avoids using the newest value until it has survived the valid-history mechanism.

### Packet transmit behavior

After the FFT decision, `process_udp()` decides the RSSI to transmit:

```c
if (AppConfig.CORType == 1) rssiheld = rssi = 255;

#ifdef DSPBEW
    if (qualnoise || (!HasCOR())) rssiheld = rssi;
#else
    rssiheld = rssi;
#endif
```

Meaning:

1. If COR is configured to be ignored, RSSI is forced to 255.
2. In a DSPBEW build, the outgoing RSSI follows live `rssi` only when noise is qualified or COR is absent.
3. If COR is present and the FFT says baseband audio is contaminating RSSI, `rssiheld` remains held.
4. In a non-DSPBEW build, `rssiheld` always follows `rssi`.

When sending a receive audio packet:

```c
if ((rssiheld > 0) && HasCOR() && HasCTCSS())
{
    UDPPut(rssiheld);
    send audio bytes;
}
else
{
    UDPPut(0);
    send silence bytes;
}
```

The actual audio samples are not modified by BEW when a signal is valid. BEW modifies the RSSI byte that accompanies the audio packet. If RSSI is zero or COR/CTCSS is not qualified, the existing packet code sends silence.

## Full algorithm pseudocode

This pseudocode is intentionally close to the firmware behavior rather than a cleaned-up redesign.

```c
// Constants
N = 32;
LOG2_N = 5;
TOP_BUCKET = 16;
MAX_RESULT = 10;
QUALCOUNT = 4;

// Persistent state
uint32_t fftresult;
uint32_t vnoise32;
uint32_t lastvnoise32[3];
uint8_t rssi;
uint8_t rssiheld;
uint8_t qualcnt = 255;
uint16_t mynoise;      // static in secondary loop
bool wascor;
bool sql2;

// Startup
twiddleFactors = TwidFactorInit(LOG2_N, conjFlag = 0);

// Per completed 20 ms audio frame, just before packet send
if (audio_frame_is_filled_and_ready()) {
    for (i = 0; i < N; i++) {
        int16_t x = ulawtabletx[completed_audio_buffer[i]];

        if (BEWMode > 1) {
            x = clamp(x, -16383, 16383);
            sig[i] = complex_q15(x, 0);
        } else {
            sig[i] = complex_q15(x / 2, 0);
        }
    }

    FFTComplexIP(LOG2_N, sig, twiddleFactors);
    BitReverseComplex(LOG2_N, sig);
    mag = SquareMagnitudeCplx(sig);

    fftresult = 0;
    for (i = 0; i < TOP_BUCKET; i++) {
        if (i >= 2) {
            fftresult += mag[i];
        }
    }

    qualnoise = (fftresult <= MAX_RESULT);
    if (BEWMode == 0) {
        qualnoise = true;
    }

    if (!qualnoise) {
        vnoise32 = lastvnoise32[0];
        lastvnoise32[1] = lastvnoise32[0];
        lastvnoise32[2] = lastvnoise32[0];
        rssiheld = calcrssi(vnoise32 >> 3);
    }

    if (CORType == IGNORE_COR) {
        rssiheld = rssi = 255;
    }

    if (qualnoise || !HasCOR()) {
        rssiheld = rssi;
    }

    send_packet_with_rssiheld_if_COR_and_CTCSS();

    if (qualnoise) {
        lastvnoise32[0] = lastvnoise32[1];
        lastvnoise32[1] = lastvnoise32[2];
        lastvnoise32[2] = vnoise32;
    }
}

// Roughly every 33 RX samples, about every 4.125 ms
if (sqlcount >= 33) {
    service_squelch(raw_diode, squelch_setting, raw_noise_adc, cal, wvf, iscaled);
    sql2 = !sql2;

    qualcor = HasCOR() && HasCTCSS();

    qualnoise = (fftresult <= MAX_RESULT) || !qualcor;
    if (BEWMode == 0) {
        qualnoise = true;
    }

    if (!qualnoise) {
        qualcnt = 0;
    }

    if (qualcnt <= QUALCOUNT) {
        qualcnt++;
        qualnoise = false;
    }

    if (sql2) {
        if (qualcor && !wascor) {
            vnoise32 = raw_noise_adc << 3;
            lastvnoise32[0] = vnoise32;
            lastvnoise32[1] = vnoise32;
            lastvnoise32[2] = vnoise32;
        } else if (qualnoise) {
            vnoise32 = ((vnoise32 * 3) + (raw_noise_adc << 3)) >> 2;
        }

        wascor = qualcor;
    }

    if (qualnoise) {
        mynoise = lastvnoise32[1];
    } else {
        // Leave mynoise unchanged. This is part of the hold.
    }

    rssi = calcrssi(mynoise >> 3);

    if (rssi < 1 && qualcor) {
        rssiheld = rssi = 1;
    }

    if (!SqlNoiseGain) {
        rssiheld = rssi = 0;
    }
}
```

## How DSP/BEW differs from the normal method

| Behavior | Normal/non-BEW | DSP/BEW enabled |
| --- | --- | --- |
| Signal-strength input | Noise/RSSI ADC channel only. | Noise/RSSI ADC channel, but accepted only when baseband FFT says the measurement is not contaminated. |
| Audio analysis | None. | 32-sample FFT over first 4 ms of each completed 20 ms audio frame. |
| Frequency content checked | Not applicable. | Sums FFT squared magnitudes for bins 2-15, approximately 500-3750 Hz at 8 kHz sampling. |
| RSSI smoothing | `vnoise32 = 3/4 old + 1/4 current`. | Same smoothing, but only when `qualnoise` is true. |
| Packet RSSI | `rssiheld = rssi`. | `rssiheld` follows `rssi` only when noise qualifies; otherwise it holds the previous valid RSSI. |
| History | No BEW history. | Three-entry `lastvnoise32[]` valid-noise history. Contamination collapses history to the oldest valid value. |
| Recovery after contamination | Immediate, because there is no contamination concept. | Delayed by `QUALCOUNT` service passes plus history behavior, roughly tens of milliseconds. |
| Squelch/COR | Raw noise squelch state machine. | Same raw squelch state machine; BEW does not replace squelch. |
| Audio payload | Sent when normal packet gates allow it. | Same audio payload when normal packet gates allow it; BEW changes RSSI, not audio content. |

## Design intent and receiver behavior

The design assumes:

1. RSSI/noise is reliable when the receiver is quiet or when baseband audio energy is low.
2. RSSI/noise may be unreliable during strong normal audio, especially high-frequency/high-deviation speech.
3. The voter server should not see those short unreliable RSSI excursions.
4. Holding the last known-good RSSI for short contaminated periods is better than reporting a false live RSSI.

This is especially relevant for receivers whose discriminator path or filtering does not deliver a stable ultrasonic/high-frequency noise component under modulation.

The feature should not be enabled for receivers that do not need it. On a receiver with clean discriminator noise behavior, DSP/BEW can hide real rapid changes in signal strength during speech because it intentionally freezes RSSI under high baseband energy.

## Implementation caveats

### The bin comments are stale

The source comments say:

```c
#define FFT_TOP_SAMPLE_BUCKET 16 // 3000 Hz
// Get the total energy above CTCSS and below 2000 Hz
```

At the documented 8 kHz sample rate and 32-point FFT, the implemented bins are 500-3750 Hz. Reimplementation should either:

1. Preserve the exact bin behavior and threshold for compatibility, or
2. Deliberately choose a corrected frequency range and then recalibrate `FFT_MAX_RESULT`.

Do not silently "fix" the comments while keeping threshold 10; the threshold is tied to the implemented range, scaling, FFT library, and magnitude representation.

### `BEWMode` is declared as `BOOL` but used as 0/1/2

`StackTsk.h` declares:

```c
BOOL BEWMode;
```

but the menu accepts values up to 2, and the FFT input path tests:

```c
if (AppConfig.BEWMode > 1)
```

Reimplement as an integer or enum with at least values 0, 1, and 2.

### ADPCM interaction is questionable

The audio comments say u-law is the default unless ADPCM is requested by host configuration. The BEW FFT path always does:

```c
x = ulawtabletx[audio_buf[filling_buffer ^ 1][i]];
```

If the buffer contains ADPCM bytes, that conversion does not represent the actual audio waveform. A faithful port should preserve this if compatibility is more important than cleanup. A clean reimplementation should feed BEW from linear PCM before codec encoding, or decode/inspect the correct codec representation.

### DSP library scaling affects the threshold

`FFTComplexIP` scales by 1/N. `SquareMagnitudeCplx` writes 16-bit fractional squared magnitudes. `FFT_MAX_RESULT = 10` only makes sense with:

1. The same 32-point FFT.
2. The same Q15 input scaling.
3. The same 1/N FFT scaling.
4. The same magnitude representation.
5. The same bin range.

Using a floating-point FFT without matching normalization will produce very different numeric magnitudes. Reimplementation should normalize to a compatible energy scale or retune the threshold empirically.

### BEW is not a squelch replacement

`service_squelch()` still receives the raw noise ADC reading before BEW gating. BEW protects the packet RSSI value; it does not stop the squelch state machine from seeing raw noise changes.

### Current compiled-but-disabled behavior is not identical to no-DSPBEW compile

With `DSPBEW` compiled and `BEWMode = 0`, the code forces `qualnoise = true`, so it should not hold RSSI due FFT contamination. However, because the DSPBEW path is still compiled, `secondary_processing_loop()` uses the `lastvnoise32[1]` history path when `qualnoise` is true. A true non-DSPBEW compile uses `mynoise = vnoise32` directly. This is a small but real behavioral distinction.

For a future implementation, decide whether "BEW disabled" should mean "bit-for-bit same as non-DSPBEW" or "same as current firmware with BEWMode 0."

## Reimplementation checklist

To recreate the current firmware behavior:

1. Keep the normal noise ADC sampling and RSSI conversion path.
2. Keep `vnoise32` as a noise value scaled left by 3 bits.
3. Keep a three-entry history of accepted `vnoise32` values.
4. Run a 32-point FFT over the first 32 samples of each 160-sample transmit frame.
5. Use the same input source as the current firmware if compatibility is required: u-law byte to `ulawtabletx[]` to signed linear.
6. Use mode 1 scaling as `x / 2`.
7. Use mode 2 scaling as clamp to +/-16383, then no division.
8. Compute squared magnitudes after FFT.
9. Sum bins 2 through 15.
10. Treat the noise sample as valid when the sum is <= 10.
11. Treat all noise samples as valid when BEWMode is 0.
12. Treat noise as valid when there is no qualified COR/CTCSS in the secondary loop.
13. Extend a bad/contaminated decision with `QUALCOUNT = 4`.
14. Only update `vnoise32` when the sample is qualified.
15. On first qualified COR, initialize all history slots to the current noise ADC value.
16. When contamination is detected at packet time, restore `vnoise32` and all history slots to the oldest valid history slot.
17. Send `rssiheld` instead of live `rssi` in audio packets.
18. Do not insert contaminated `vnoise32` values into the valid history.

## Suggested cleaner architecture for a future port

The current firmware is tightly optimized for a dsPIC and old Microchip libraries. A future reimplementation could keep the behavior but clarify the structure:

```text
AudioSampler
  produces linear PCM at 8 kHz

NoiseSampler
  produces raw 10-bit receiver noise ADC values

BewDetector
  input: 32-sample PCM window
  output: audio_contaminated boolean

RssiTracker
  input: raw noise, qualcor, audio_contaminated
  output: live_rssi, held_rssi

PacketSender
  sends held_rssi with the audio payload
```

Recommended explicit API:

```c
typedef enum {
    BEW_OFF = 0,
    BEW_NORMAL = 1,
    BEW_HIGH_SENSITIVITY = 2
} BewMode;

typedef struct {
    BewMode mode;
    uint16_t max_energy;
    uint8_t hold_count;
    uint32_t vnoise32;
    uint32_t valid_noise_history[3];
    uint32_t last_fft_energy;
    uint8_t live_rssi;
    uint8_t held_rssi;
    uint8_t post_bad_hold_counter;
    bool was_qualcor;
} BewRssiState;
```

Separate "detect contamination" from "hold RSSI." That makes it easier to unit-test the FFT threshold and the RSSI hold state machine independently.

## Test cases for a future implementation

A future implementation should have tests for:

1. BEWMode 0 never marks a frame contaminated.
2. Silent/near-silent PCM produces `fftresult <= 10` under the chosen normalization.
3. A 1000 Hz tone at sufficient amplitude produces `fftresult > 10`.
4. Bins 0 and 1 do not contribute to the sum.
5. Bin 16 does not contribute to the sum.
6. Mode 2 produces higher energy than mode 1 for the same unclipped input.
7. Contamination prevents `vnoise32` from updating.
8. Contamination holds `rssiheld` at the previous accepted value.
9. Good frames shift `lastvnoise32[]` history.
10. First qualified COR initializes all three history slots.
11. `QUALCOUNT` extends hold after contamination clears.
12. No-COR condition allows noise updates even if the last FFT result was bad.
13. `SqlNoiseGain == 0` forces RSSI to zero.
14. COR ignore mode forces RSSI to 255.

## Minimal compatible pseudocode for the detector only

If all that is needed is the DSP/BEW contamination detector:

```c
bool bew_noise_is_qualified(uint8_t ulaw[160], int mode)
{
    if (mode == 0) {
        return true;
    }

    complex_q15 sig[32];

    for (int i = 0; i < 32; i++) {
        int16_t x = ulawtabletx[ulaw[i]];

        if (mode > 1) {
            x = clamp(x, -16383, 16383);
            sig[i] = complex_q15(x, 0);
        } else {
            sig[i] = complex_q15(x / 2, 0);
        }
    }

    fft32_scaled_like_microchip(sig);
    bit_reverse_to_natural_order(sig);

    uint32_t energy = 0;
    for (int k = 2; k < 16; k++) {
        energy += square_magnitude_q15_to_uint16(sig[k]);
    }

    return energy <= 10;
}
```

If using a modern floating-point FFT, the equivalent detector should define a new threshold after matching or replacing the original normalization.

## Practical interpretation

DSP/BEW is a short-duration "do not trust RSSI right now" detector. It assumes that large baseband audio energy is a proxy for moments when receiver noise-derived RSSI is unreliable. During those moments, the firmware keeps sending valid audio but freezes the RSSI byte at the last accepted value. This prevents the host voter from making voting decisions based on modulation-induced RSSI artifacts rather than actual RF signal quality.

The most important compatibility details are:

1. 32 samples at 8 kHz.
2. u-law table to signed linear input.
3. mode 1 divides by 2, mode 2 clips to +/-16383.
4. Microchip-style scaled FFT.
5. squared magnitude bins 2-15.
6. threshold <= 10 means qualified/usable.
7. bad windows hold `vnoise32`/`rssiheld` and do not enter history.
