<CsoundSynthesizer>
<CsOptions>
-odac0      -d     ;;;RT audio I/O
</CsOptions>
<CsInstruments>
sr	=  44100
kr	=  441
nchnls	=  2

chn_k "Pulse1", 3
chn_k "Pulse2", 3
chn_k "Pulse3", 3
chn_k "Pulse4", 3
chn_k "Pulse5", 3
chn_k "Pulse6", 3

chn_k "fof1", 3
chn_k "fof2", 3
chn_k "fof3", 3
chn_k "fof4", 3

chn_k "Bass1", 3
chn_k "Bass2", 3
chn_k "Bass3", 3

chn_k "Solo01", 3
chn_k "Solo02", 3
chn_k "Solo03", 3

chn_k "Solo11", 3
chn_k "Solo12", 3
chn_k "Solo13", 3


          
/* ======== drum instruments by Istvan Varga, Mar 10 2002 ======== */

	seed 0
;=====================================
; REVERB INITIALIZATION
;=====================================

garvbsig  init      0

;=====================================
; DELAY INITIALIZATION
;=====================================

gasig     init      0
/* ----------------------- global variables ----------------------- */

giflen	=  131072	/* table length in samples	*/
giovr	=  4		/* oversample			*/
gibwd	=  24000	/* max. bandwidth in Hz		*/
gitmpfn	=  99		/* tmp ftable number		*/

; spatializer parameters

gisptd	=  0.5		/* unit circle distance (see spat3d manual)  */
gisptf	=  225		/* room table number (0: no room simulation) */
gisptm	=  2		/* spat3di mode (see manual)		     */

gisptx	=  0.2		/* extra time for room echoes		     */

gidsts	=  0.35		/* distance scale			     */

ga0	init 0		; mono output
ga1	init 0		; spat3di out 1
ga2	init 0		; spat3di out 2
ga3	init 0		; spat3di out 3
ga4	init 0		; spat3di out 4

; mono output file name (for external convolve unit)

#define SNDFL_MONO # "mono_out.pcm" #

/* ---------------------- some useful macros ---------------------- */

; spatialize and send output

#define SPAT_OUT #

a1	rnd31 0.000001 * 0.000001 * 0.000001 * 0.000001, 0, 0
a0	=  a0 + a1

iX	=  iX * gidsts
iY	=  iY * gidsts
iZ	=  iZ * gidsts

a1, a2, a3, a4	spat3di a0, iX, iY, iZ, gisptd, gisptf, gisptm

	vincr ga0, a0
	vincr ga1, a1
	vincr ga2, a2
	vincr ga3, a3
	vincr ga4, a4

#

; convert velocity to amplitude

#define VELOC2AMP(VELOCITY'MAXAMP) # (($MAXAMP) * (0.0039 + ($VELOCITY) * ($VELOCITY) / 16192)) #

; convert MIDI note number to frequency

#define MIDI2CPS(NOTNUM) # (440 * exp(log(2) * (($NOTNUM) - 69) / 12)) #

; power of two number greater than x

#define POW2CEIL(P2C_X) # (int(0.5 + exp(log(2) * int(1.01 + log($P2C_X) / log(2))))) #

; semitones to frequency ratio

#define NOTE2FRQ(XNOTE) # (exp(log(2) * ($XNOTE) / 12)) #

; frequency to table number

#define CPS2FNUM(XCPS'BASE_FNUM) # int(69.5 + ($BASE_FNUM) + 12 * log(($XCPS) / 440) / log(2)) #

/* ---------------- constants ---------------- */

#define PI	# 3.14159265 #
#define TWOPI	# (2 * 3.14159265) #

; ---- instr 1: render tables for cymbal instruments ----

	instr 1

ifn	=  p4		/* ftable number		*/
inumh	=  p5		/* number of partials		*/
iscl	=  p6		/* amp. scale			*/
itrns	=  p7		/* transpose (in semitones)	*/
isd	=  p8		/* random seed (1 to 2^31 - 2)	*/
idst	=  p9		/* amplitude distribution	*/

imaxf	=  $NOTE2FRQ(itrns) * gibwd		; max. frequency
itmp	rnd31 1, 0, isd				; initialize seed

; create empty table for parameters

ifln	=  $POW2CEIL(3 * inumh)
itmp	ftgen gitmpfn, 0, ifln, -2, 0

i1	=  0
l01:
iamp	rnd31 1, idst, 0	; amplitude
icps	rnd31 imaxf, 0, 0	; frequency
iphs	rnd31 1, 0, 0		; phase
iphs	=  abs(iphs)
; cut off partials with too high frequency
iamp	=  (icps > (sr * 0.5) ? 0 : iamp)
iphs	=  (icps > (sr * 0.5) ? 0 : iphs)
icps	=  (icps > (sr * 0.5) ? 0 : icps)
; write params to table
	tableiw iamp, i1 * 3 + 0.25, gitmpfn
	tableiw icps, i1 * 3 + 1.25, gitmpfn
	tableiw iphs, i1 * 3 + 2.25, gitmpfn
i1	=  i1 + 1
	if (i1 < (inumh - 0.5)) igoto l01

; render table

ifln	=  giflen * giovr + 0.25	; length with oversample
itmp	ftgen ifn, 0, ifln, -33, gitmpfn, inumh, iscl, -(giovr)

	endin


/* ---- instr 10: cymbal ---- */

	instr 10

ipan = p6
ilnth	=  p3		/* note length				     */
ifn	=  12		/* function table with instrument parameters */
ivel	=  p5		/* velocity (0 - 127)			     */

iscl	table  0, ifn	; amplitude scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ixfn	table  6, ifn	; input table
iwfn	table  7, ifn	; window table
igdurs	table  8, ifn	; start grain duration in seconds
igdurt	table  9, ifn	; grain duraton envelope half-time
igdure	table 10, ifn	; end grain duration
iovrlp	table 11, ifn	; number of overlaps
iEQfs	table 12, ifn	; EQ start frequency
iEQft	table 13, ifn	; EQ frequency envelope half-time
iEQfe	table 14, ifn	; EQ end frequency
iEQls	table 15, ifn	; EQ start level (Q is level * 0.7071)
iEQlt	table 16, ifn	; EQ level envelope half-time
iEQle	table 17, ifn	; EQ end level
ihpf	table 18, ifn	; highpass frequency
ilpf	table 19, ifn	; lowpass frequency
idec	table 20, ifn	; decay env. half-time (n.a. in reverse mode)
irvmod	table 21, ifn	; reverse cymbal mode (0: on, 1: off)
idel2	table 22, ifn	; delay time for chorus effect
ilvl1	table 23, ifn	; non-delayed signal level
ilvl2	table 24, ifn	; delayed signal level

ixtime	=  gisptx + idel + irel + idel2		; expand note duration
p3	=  p3 + ixtime

; release envelope

aenv1	linseg 1, ilnth, 1, irel, 0, 1, 0
aenv1	=  aenv1 * aenv1

; output amplitude
iamp	=  $VELOC2AMP(ivel'iscl)
; grain duration
kgdur	port igdure, igdurt, igdurs
; 4 * sr = 192000Hz (sample rate of input file)
a1	grain3	giovr * sr / ftlen(ixfn), 0.5, 0, 0.5,  kgdur, iovrlp / kgdur, iovrlp + 2,	ixfn, iwfn, 0, 0, 0, 16

; filters

kEQf	port iEQfe, iEQft, iEQfs
kEQl	port iEQle, iEQlt, iEQls
a1	pareq a1, kEQf, kEQl, kEQl * 0.7071, 0
a1	butterhp a1, ihpf
a1	butterlp a1, ilpf

; amp. envelope

aenv2	expon 1, idec, 1 - 0.5 * irvmod
aenv3	linseg irvmod, ilnth, 1, 1, 1
a1	=  a1 * iamp * aenv1 * aenv2 * (aenv3 * aenv3)

; delays

a2	delay a1 * ilvl2, idel2
a0	delay a2 + a1 * ilvl1, idel

outs      a0*sqrt(ipan),a0*sqrt(1-ipan)

garvbsig  =         garvbsig+(a0*.001)

;$SPAT_OUT

	endin

/* ---------------------- instr 20: bass drum ---------------------- */

	instr 20

; +------------+             +------------+     +------------+
; | oscillator |--->---+-->--| highpass 1 |-->--| bandpass 1 |-->--+
; +------------+       |     +------------+     +------------+     |
;                      |                                           V
;          +-----<-----+                        +-------------+    |
;          |           |              +----<----| "allpass" 1 |----+
;          |           V              |         +-------------+
;          |           |              |
;          |    +------------+      +---+       +-----------------+
;          |    | highpass 2 |      | + |--->---| output highpass |--+
;          |    +------------+      +---+       +-----------------+  |
;          |           |              |                              V
;          |           V              ^         +----------------+   |
;          |           |              |         | output lowpass |-<-+
;          |         +---+     +------------+   +----------------+
;          +---->----| + |-->--| highpass 3 |           |
;                    +---+     +------------+           +---->-----+
;                                                                  |
; +-----------------+   +----------------+   +---------------+   +---+ output
; | noise generator |->-| noise bandpass |->-| noise lowpass |->-| + |-------->
; +-----------------+   +----------------+   +---------------+   +---+

ilnth	=  p3		; note length
ifn	=  p4		; table number
ivel	=  p5		; velocity

iscl	table  0, ifn	; volume
idel	table  1, ifn	; delay (in seconds)
irel	table  2, ifn	; release time (sec.)
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z

ibpm	table  6, ifn	; tempo

ibsfrq	table  7, ifn	; base frequency (MIDI note number)
ifrqs	table  8, ifn	; oscillator start frequency / base frequency
ifrqt	table  9, ifn	; oscillator freq. envelope half-time in beats

ibw01	table 10, ifn	; bandpass 1 bandwidth / oscillator frequency
ihp1	table 11, ifn	; highpass 1 freq. / oscillator frequency
iapf1	table 12, ifn	; "allpass" 1 start freq. / oscillator frq.
iapdx	table 13, ifn	; "allpass" 1 envelope half-time in beats
iapf2	table 14, ifn	; "allpass" 1 end frq. / oscillator frequency

ihp2	table 15, ifn	; highpass 2 frequency / base frequency
imx2	table 16, ifn	; highpass 2 output gain
ihp3	table 17, ifn	; highpass 3 freq. / base frequency
imx3	table 18, ifn	; highpass 3 output gain

ihpx	table 19, ifn	; output highpass frequency / base frequency
iq0x	table 20, ifn	; output highpass resonance

ifr1	table 21, ifn	; output lowpass start freq 1 / oscillator frq.
ifdx1	table 22, ifn	; output lowpass frequency 1 half-time in beats
ifr2	table 23, ifn	; output lowpass start freq 2 / oscillator frq.
ifdx2	table 24, ifn	; output lowpass frequency 2 half-time in beats

insbp1	table 25, ifn	; noise bandpass start frequency in Hz
insbp2	table 26, ifn	; noise bandpass end frequency in Hz
insbw	table 27, ifn	; noise bandpass bandwidth / frequency
inslp1	table 28, ifn	; noise lowpass start frequency (Hz)
inslp2	table 29, ifn	; noise lowpass end frequency (Hz)
insht	table 30, ifn	; noise filter envelope half-time in beats
insatt	table 31, ifn	; noise attack time (in seconds)
insdec	table 32, ifn	; noise decay half-time (in beats)
insmx	table 33, ifn	; noise mix

ixtim	=  gisptx + idel + irel		; expand note length
p3	=  p3 + ixtim
; note amplitude
iamp	=  $VELOC2AMP(ivel'iscl)
; release envelope
aenv	linseg 1, ilnth, 1, irel, 0, 1, 0
aenv	=  aenv * aenv
; beat time
ibtime	=  60 / ibpm

; ---- noise generator ----

a_ns	rnd31 32768 * insmx, 0, 0
k_nsf	expon 1, ibtime * insht, 0.5
k_nsbp	=  insbp2 + (insbp1 - insbp2) * k_nsf
k_nslp	=  inslp2 + (inslp1 - inslp2) * k_nsf
; noise bandpass
a_ns	butterbp a_ns, k_nsbp, k_nsbp * insbw
; noise lowpass
a_ns	pareq a_ns, k_nslp, 0, 0.7071, 2
; noise amp. envelope
a_nse1	linseg 0, insatt, 1, 1, 1
a_nse2	expon 1, ibtime * insdec, 0.5
a_ns	=  a_ns * a_nse1 * a_nse2

; ---- oscillator ----

; base frequency
icps	=  $MIDI2CPS(ibsfrq)
; oscillator frequency
kfrq	expon 1, ibtime * ifrqt, 0.5
kfrq	=  icps * (1 + (ifrqs - 1) * kfrq)
; table number
kfn	=  $CPS2FNUM(kfrq'300)
a1	phasor kfrq
a2	tablexkt a1, kfn, 0, 2, 1, 0, 1
a1	=  a2 * 16384
a2	=  a1				; a1 = a2 = osc. signal

; ---- filters ----

; highpass 1
a1	butterhp a1, ihp1 * kfrq
; bandpass 1
a1	butterbp a1, kfrq, ibw01 * kfrq
; "allpass" 1
k_apf	expon 1, ibtime * iapdx, 0.5
k_apf	=  (iapf2 + (iapf1 - iapf2) * k_apf) * kfrq
atmp	tone a1, k_apf
a1	=  2 * atmp - a1
; highpass 2
a3	butterhp a2, ihp2 * icps
; highpass 3
a2	butterhp a2 + a3 * imx2, ihp3 * icps
a1	=  a1 + a2 * imx3
; output highpass
a1	pareq a1, ihpx * icps, 0, iq0x, 1
; output lowpass
k1	expon 1, ibtime * ifdx1, 0.5
k2	expon 1, ibtime * ifdx2, 0.5
kfrx	limit (k1 * ifr1 + k2 * ifr2) * kfrq, 10, sr * 0.48
a1	pareq a1, kfrx, 0, 0.7071, 2

a0	delay (a1 + a_ns) * iamp * aenv, idel

$SPAT_OUT

	endin

/* ------------------ instr 21: TR-808 bass drum ------------------ */

	instr 21

ilnth	=  p3		; note length
ifn	=  p4		; table number
ivel	=  p5		; velocity

iscl	table  0, ifn	; amp. scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ibsfrq	table  6, ifn	; base frequency (MIDI note)
ifrqs	table  7, ifn	; start frequency / base frq
ifrqt	table  8, ifn	; frequency envelope half-time
iphs	table  9, ifn	; start phase (0..1)
ilpfrq	table 10, ifn	; lowpass filter frequency
idect	table 11, ifn	; decay half-time

ixtim	=  gisptx + idel + irel		; expand note length
p3	=  p3 + ixtim
; note amplitude
iamp	=  $VELOC2AMP(ivel'iscl)

icps	=  $MIDI2CPS(ibsfrq)
kcps	port 1, ifrqt, ifrqs
kcps	=  icps * kcps

a1	oscili 1, kcps, 700, iphs
a1	butterlp a1, ilpfrq

aenv	expon 1, idect, 0.5
aenv2	linseg 0, 0.02, 1, ilnth-irel-0.02, 1, irel, 0, 1, 0

a0	delay a1 * iamp * aenv * (aenv2 * aenv2), idel

$SPAT_OUT

	endin

/* ---------------------- instr 30: hand clap ---------------------- */

	instr 30

ilnth	=  p3		; note length
ifn	=  p4		; table number
ivel	=  p5		; velocity

iscl	table  0, ifn	; amp. scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ibpfrq	table  6, ifn	; bandpass frequency
ibws	table  7, ifn	; bandwidth envelope start
ibwt	table  8, ifn	; bw. envelope half-time
ibwe	table  9, ifn	; bandwidth envelope end
idel2	table 10, ifn	; delay 2
idel3	table 11, ifn	; delay 3
idel4	table 12, ifn	; delay 4
idec1	table 13, ifn	; decay 1
idec2	table 14, ifn	; decay 2
idec3	table 15, ifn	; decay 3
idec4	table 16, ifn	; decay 4

ixtim	=  gisptx + idel + irel		; expand note length
p3	=  p3 + ixtim
; note amplitude
iamp	=  $VELOC2AMP(ivel'iscl)
; bandwidth envelope
kbwd	port ibwe, ibwt, ibws
; amp. envelope
a1	=  1
a2	delay1 a1
a1	=  a1 - a2
a2	delay a1, idel2
a3	delay a1, idel3
a4	delay a1, idel4
a1	tone a1 * idec1, 1 / idec1
a2	tone a2 * idec2, 1 / idec2
a3	tone a3 * idec3, 1 / idec3
a4	tone a4 * idec4, 1 / idec4
; noise generator with bandpass filter
a0	rnd31 iamp, 0, 0
a0	butterbp a0 * (a1 + a2 + a3 + a4), ibpfrq, kbwd
; release envelope and delay
a1	linseg 1, ilnth, 1, irel, 0, 1, 0
a1	=  a1 * a1 * a0
a0	delay a1, idel

$SPAT_OUT

	endin

/* ------------------- instr 31: TR-808 cowbell ------------------- */

	instr 31

ilnth	=  p3		; note length
ifn	=  p4		; table number
ivel	=  p5		; velocity

iscl	table  0, ifn	; amp. scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ifrq1	table  6, ifn	; frequency 1
ifrq2	table  7, ifn	; frequency 2
iffrqs	table  8, ifn	; lowpass filter start frequency
iffrqt	table  9, ifn	; lowpass filter envelope half-time
iffrqe	table 10, ifn	; lowpass filter end frequency
iatt	table 11, ifn	; attack time
idect1	table 12, ifn	; decay time 1
idecl1	table 13, ifn	; decay level 1
idect2	table 14, ifn	; decay 2 half-time
iresn	table 15, ifn	; resonance at osc2 frequency

ixtim	=  gisptx + idel + irel		; expand note length
p3	=  p3 + ixtim
; note amplitude
iamp	=  $VELOC2AMP(ivel'iscl)

ifrq1	=  $MIDI2CPS(ifrq1)
ifn1	=  $CPS2FNUM(ifrq1'500)
ifrq2	=  $MIDI2CPS(ifrq2)
ifn2	=  $CPS2FNUM(ifrq2'500)

a1	oscili 1, ifrq1, ifn1
a2	oscili 1, ifrq2, ifn2

kffrq	port iffrqe, iffrqt, iffrqs
kffrq	limit kffrq, 10, sr * 0.48

aenv1	linseg 0, iatt, 1, 1, 1				; attack
aenv2	expseg 1, idect1, idecl1, idect2, idecl1 * 0.5	; decay
aenv3	linseg 1, ilnth, 1, irel, 0, 1, 0		; release

a0	tone a1 + a2, kffrq
a1	pareq a0, ifrq2, iresn, iresn, 0

a0	delay a1 * iamp * aenv1 * aenv2 * (aenv3 * aenv3), idel

$SPAT_OUT

	endin

/* -------------------- instr 40: TR-808 hi-hat -------------------- */

	instr 40

ilnth	=  p3		; note length
ifn	=  p4		; table number
ivel	=  p5		; velocity

iscl	table  0, ifn	; amp. scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ibsfrq	table  6, ifn	; base frequency (MIDI note)
ifrq2	table  7, ifn	; osc 2 frequency / base frq
ifrq3	table  8, ifn	; osc 3 frequency / base frq
ifrq4	table  9, ifn	; osc 4 frequency / base frq
ifrq5	table 10, ifn	; osc 5 frequency / base frq
ifrq6	table 11, ifn	; osc 6 frequency / base frq
idsts	table 12, ifn	; distortion start
idstt	table 13, ifn	; distortion envelope half-time
idste	table 14, ifn	; distortion end
ihpfrq	table 15, ifn	; highpass frequency
ihpres	table 16, ifn	; highpass resonance
iatt	table 17, ifn	; attack time
idect1	table 18, ifn	; decay time 1
idecl1	table 19, ifn	; decay level 1
idect2	table 20, ifn	; decay 2 half-time

ixtim	=  gisptx + idel + irel		; expand note length
p3	=  p3 + ixtim
; note amplitude
iamp	=  $VELOC2AMP(ivel'iscl)

ifrq1	=  $MIDI2CPS(ibsfrq)		; oscillator frequencies
ifrq2	=  ifrq1 * ifrq2
ifrq3	=  ifrq1 * ifrq3
ifrq4	=  ifrq1 * ifrq4
ifrq5	=  ifrq1 * ifrq5
ifrq6	=  ifrq1 * ifrq6

ifn1	=  $CPS2FNUM(ifrq1'300)		; table numbers
ifn2	=  $CPS2FNUM(ifrq2'300)
ifn3	=  $CPS2FNUM(ifrq3'300)
ifn4	=  $CPS2FNUM(ifrq4'300)
ifn5	=  $CPS2FNUM(ifrq5'300)
ifn6	=  $CPS2FNUM(ifrq6'300)

iphs1	unirand 1			; start phase
iphs2	unirand 1
iphs3	unirand 1
iphs4	unirand 1
iphs5	unirand 1
iphs6	unirand 1

a1	oscili 1, ifrq1, ifn1, iphs1	; oscillator
a2	oscili 1, ifrq2, ifn2, iphs2
a3	oscili 1, ifrq3, ifn3, iphs3
a4	oscili 1, ifrq4, ifn4, iphs4
a5	oscili 1, ifrq5, ifn5, iphs5
a6	oscili 1, ifrq6, ifn6, iphs6

a0	=  a1 + a2 + a3 + a4 + a5 + a6

a1	limit a0 * 1000000, -1, 1			; distort
a0	limit abs(a0), 0.000001, 1000000
adst	expon 1, idstt, 0.5
a0	=  a1 * exp(log(a0) * (idste + (idsts - idste) * adst))

a0	pareq a0, ihpfrq, 0, sqrt(ihpres), 1		; highpass
a0	pareq a0, ihpfrq, 0, sqrt(ihpres), 1

aenv1	linseg 0, iatt, 1, 1, 1				; envelopes
aenv2	expseg 1, idect1, idecl1, idect2, 0.5 * idecl1
aenv3	linseg 1, ilnth, 1, irel, 0, 1, 0

a1	=  a0 * iamp * aenv1 * aenv2 * (aenv3 * aenv3)

a0	delay a1, idel

$SPAT_OUT

	endin

/* ------------------ instr 50: TR-909 snare drum ------------------ */

	instr 50

ilnth	=  p3		/* note length				     */
ifn	=  p4		/* function table with instrument parameters */
ivel	=  p5		/* velocity (0 - 127)			     */

iscl	table  0, ifn	; amplitude scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ibsfrq	table  6, ifn	; base freq. (MIDI note)
ifrqs	table  7, ifn	; start freq. / base frq.
ifrqt	table  8, ifn	; frequency env. half-time
ifmds	table  9, ifn	; FM depth start
ifmdt	table 10, ifn	; FM depth envelope half-time
ifmde	table 11, ifn	; FM depth end
ifrq2	table 12, ifn	; osc 2 frq. / osc 1 frq.
iamp2s	table 13, ifn	; osc 2 amplitude start
iamp2t	table 14, ifn	; osc 2 amplitude envelope half-time
iamp2e	table 15, ifn	; osc 2 amplitude end
insbpf	table 16, ifn	; noise BP frequency
insbpb	table 17, ifn	; noise BP bandwidth
insamps	table 18, ifn	; noise amplitude start
insampt	table 19, ifn	; noise amplitude env. half-time
insampe	table 20, ifn	; noise amplitude end
idect	table 21, ifn	; decay half-time

ixtime	=  gisptx + idel + irel			; expand note duration
p3	=  p3 + ixtime

; release envelope

aenv1	linseg 1, ilnth, 1, irel, 0, 1, 0
aenv1	=  aenv1 * aenv1

; output amplitude
iamp	=  $VELOC2AMP(ivel'iscl)

icps0	=  $MIDI2CPS(ibsfrq)	; frequency envelope
icps1	=  ifrqs * icps0
acps	expon 1, ifrqt, 0.5
acps	=  icps0 + (icps1 - icps0) * acps	; osc 1 frequency
acps2	=  acps * ifrq2				; osc 2 frequency

afmd	expon 1, ifmdt, 0.5		; FM depth
afmd	=  ifmde + (ifmds - ifmde) * afmd

afm1	oscili afmd, acps, 700		; FM
afm2	oscili afmd, acps2, 700

aamp2	expon 1, iamp2t, 0.5		; osc 2 amplitude
aamp2	=  iamp2e + (iamp2s - iamp2e) * aamp2

a1	oscili 1, acps * (1 + afm1), 700	; oscillators
a2	oscili aamp2, acps2 * (1 + afm2), 700

a3	rnd31 1, 0, 0			; noise
aamp3	expon 1, insampt, 0.5
a3	butterbp a3, insbpf, insbpb
a3	=  a3 * (insampe + (insamps - insampe) * aamp3)

aenv2	expon 1, idect, 0.5
a1	=  iamp * aenv1 * aenv2 * (a1 + a2 + a3)

a0	delay a1, idel

$SPAT_OUT

	endin

/* ------------------------- instr 51: tom ------------------------- */

	instr 51

ipan = p6
ilnth	=  p3		/* note length				     */
ifn	=  58		/* function table with instrument parameters */
ivel	=  p5*0.7		/* velocity (0 - 127)			     */

iscl	table  0, ifn	; amplitude scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ibsfrq 	table  6, ifn	; base freq. (MIDI note)
ifrqs	table  7, ifn	; start freq. / base frq.
ifrqt	table  8, ifn	; frequency env. half-time
ifrq2	table  9, ifn	; osc 2 frq. / osc 1 frq.
iamp2s	table 10, ifn	; osc 2 amplitude start
iamp2t	table 11, ifn	; osc 2 amplitude envelope half-time
iamp2e	table 12, ifn	; osc 2 amplitude end
iphs	table 13, ifn	; osc 1 and 2 start phase
idel_1	table 14, ifn	; invert 1 delay * base frequency
idel_2	table 15, ifn	; invert 2 delay * base frequency
ilpfrq	table 16, ifn	; lowpass frequency
insbpf	table 17, ifn	; noise BP frequency
insbpb	table 18, ifn	; noise BP bandwidth
inscfr	table 19, ifn	; noise comb frequency / base frequency
inscfb	table 20, ifn	; noise comb feedback
inslpf	= p4;table 21, ifn	; noise lowpass frequency
insamps	table 22, ifn	; noise amplitude start
insampt	table 23, ifn	; noise amplitude env. half-time
insampe	table 24, ifn	; noise amplitude end
idect	table 25, ifn	; decay half-time

ixtime	=  gisptx + idel + irel			; expand note duration
p3	=  p3 + ixtime

; release envelope

aenv1	linseg 1, ilnth, 1, irel, 0, 1, 0
aenv1	=  aenv1 * aenv1

; output amplitude
iamp	=  $VELOC2AMP(ivel'iscl)

icps0	=  $MIDI2CPS(ibsfrq)	; frequency envelope
icps1	=  ifrqs * icps0
acps	expon 1, ifrqt, 0.5
acps	=  icps0 + (icps1 - icps0) * acps	; osc 1 frequency
acps2	=  acps * ifrq2				; osc 2 frequency

aamp2	expon 1, iamp2t, 0.5		; osc 2 amplitude
aamp2	=  iamp2e + (iamp2s - iamp2e) * aamp2

a1	oscili 1, acps, 700, iphs	; oscillators
a2	oscili aamp2, acps2, 700, iphs

a3	rnd31 1, 0, 0			; noise
aamp3	expon 1, insampt, 0.5
a3	butterbp a3, insbpf, insbpb
a3	=  a3 * (insampe + (insamps - insampe) * aamp3)

ax1	=  1		; invert amplitude to add click
ax2	delay ax1, (int(idel_1 * sr / icps0 + 0.5) + 0.01) / sr
ax3	delay ax1, (int(idel_2 * sr / icps0 + 0.5) + 0.01) / sr

a1	=  (a1 + a2) * (ax1 - 2 * ax2 + 2 * ax3)

a1	butterlp a1, ilpfrq		; lowpass

a3x	delayr (int(sr / (inscfr * icps0) + 0.5) + 0.01) / sr
a3	=  a3 - a3x * inscfb
	delayw a3

a3	butterlp a3, inslpf

aenv2	expon 1, idect, 0.5
a1	=  iamp * aenv1 * aenv2 * (a1 + a3)

outs      a1*sqrt(ipan),a1*sqrt(1-ipan)

garvbsig  =         garvbsig+(a1*.001)
;a0	delay a1, idel

;$SPAT_OUT

	endin

/* ---------------------- instr 52: rim shot ---------------------- */

	instr 52

ipan = p6
ilnth	=  p3		/* note length				     */
ifn	=  56;p4		/* function table with instrument parameters */
ivel	=  p5		/* velocity (0 - 127)			     */

iscl	table  0, ifn	; amplitude scale
idel	table  1, ifn	; delay
irel	table  2, ifn	; release time
iX	table  3, ifn	; X
iY	table  4, ifn	; Y
iZ	table  5, ifn	; Z
ibsfrq	= p4 ;table  6, ifn	; base freq. (MIDI note)
ifrqs	table  7, ifn	; start freq. / base frq.
ifrqt	table  8, ifn	; frequency env. half-time
ifmds	table  9, ifn	; FM depth start  = p4		;
ifmdt	table 10, ifn	; FM depth envelope half-time
ifmde	table 11, ifn	; FM depth end = 0.02 * p4/3;
insamp	table 12, ifn	; noise amplitude = 0.1 * p4/3;
inslpf	table 13, ifn	; noise lowpass frequency
idecs	table 14, ifn	; amplitude (before distortion) start
idect	table 15, ifn	; amplitude envelope half-time
idece	table 16, ifn	; amplitude end
ihpamp	table 17, ifn	; HP filtered signal (after distortion) gain
ihpfrq	table 18, ifn	; highpass frequency
ihpdel	table 19, ifn	; highpass filtered signal delay
ilpfs	table 20, ifn	; output lowpass frequency start
ilpft	table 21, ifn	; lowpass envelope half-time
ilpfe	table 22, ifn	; output lowpass frequency end

ixtime	=  gisptx + idel + irel		; expand note duration
p3	=  p3 + ixtime

; release envelope

aenv1	linseg 1, ilnth, 1, irel, 0, 1, 0
aenv1	=  aenv1 * aenv1

iamp	=  $VELOC2AMP(ivel'1)		; velocity

icps	=  $MIDI2CPS(ibsfrq)		; base frequency
acps	expon 1, ifrqt, 0.5
acps	=  icps * (1 + (ifrqs - 1) * acps)

a1a	phasor acps, 0			; square wave
a1b	phasor acps, 0.5

afmd	expon 1, ifmdt, 0.5		; FM envelope
afmd	=  ifmde + (ifmds - ifmde) * afmd

a1	=  (a1a - a1b) * 2 * afmd
acps	=  acps * (1 + a1)		; frequency with FM

a0	oscili 1, acps, 700		; sine oscillator

a1	rnd31 insamp, 0, 0		; add some noise
a1	tone a1, inslpf
	vincr a0, a1

aenv	expon 1, idect, 0.5		; amp. envelope
aenv	=  idece + (idecs - idece) * aenv

a0	limit aenv * iamp * a0, -1, 1	; distortion
a0	tablei a0 * 0.25, 700, 1, 0, 1

a2	tone a0, ihpfrq			; highpass filter
a2	=  a0 - a2
a1	delay a2, ihpdel
	vincr a0, a1 * ihpamp

klpfr	port ilpfe, ilpft, ilpfs	; output lowpass
a1	pareq a0, klpfr, 0, 0.7071, 2

a0	delay a1 * iscl * aenv1, idel

outs      a0*sqrt(ipan),a0*sqrt(1-ipan)

garvbsig  =         garvbsig+(a0*.002)

;$SPAT_OUT

	endin
	
/* ------------ instr 60: BD ------------ */	
	instr 60
	
  i_len = p3+0.2
  
  icps = p4
  ipan = p5
  iamp = 1
  
  kcps	expon 1, 0.022, 0.5
  kcps	=  4.3333 * kcps * icps + icps
  
  a1	phasor kcps
  a2	phasor kcps, 0.5
  a1	=  a1 - a2
  
  kffrq1  expon 1, 0.07, 0.5
  kffrq2	expon 1, 0.01, 0.5
  kffrq	=  (kffrq1 + kffrq2) * kcps

  a1	pareq a1, kffrq, 0, 0.7071, 2
  
  a1	=  taninv(a1 * 20)
  
  a1	pareq a1, kcps * 6, 2, 1, 2
  a1	pareq a1, icps * 1.25, 2.5, 1, 0
  
  a2	linseg 1, i_len, 1, 0.01, 0, 1, 0
  a1	=  a1 * a2 * iamp * 4500 + (1/1e24)
  al = a1 *sqrt(ipan)
  ar = a1 *sqrt(1-ipan)
  
  outs al,ar

endin
;---------------------------------------------------------
; Kick Drum
;---------------------------------------------------------
	instr 61

/* ------------------------------------------------------------------------- */
ipan      		=       p6
iTimbre 		= 		p4
ivol	=  2		/* volume					*/
ibpm	=  150		/* tempo					*/

irel	=  0.1		/* release time					*/

istrtf1	=  64.0		/* start frequency / note f.			*/
ifdec1	=  128		/* freq. decay speed				*/

insmix	=  1.0		/* noise mix					*/

iHP1f	=  iTimbre	/* HP filter 1 frequency / osc. freq.		*/
iBP1f	=  iTimbre		/* BP filter 1 frequency / osc. freq.		*/
iBP1b	=  1.0		/* BP filter 1 bandwidth / osc. freq.		*/

iHP2f	=  2.0		/* HP filter 2 frequency / osc. freq.		*/
iHP2m	=  0.0		/* HP filter 2 mix				*/
iHP3f	=  8.0		/* HP filter 3 frequency / osc. freq.		*/
iHP3m	=  0.0		/* HP filter 3 mix				*/

iHPxf	=  1.0		/* output HP filter frequency / note frequency	*/
iHPxr	=  0.7		/* output HP filter resonance			*/

iLPf1	=  4.0		/* output LP filter frequency 1 / osc. freq.	*/
iLPd1	=  4		/* output LP filter frequency 1 decay speed	*/
iLPf2	=  4.0		/* output LP filter frequency 2 / osc. freq.	*/
iLPd2	=  4		/* output LP filter frequency 2 decay speed	*/

iAMxd	=  8		/* amp. envelope speed				*/

iAM1s	=  0.25		/* amp. envelope 1 start value			*/
iAM1d	=  8		/* amp. envelope 1 decay speed			*/
iAM2s	=  2.0		/* amp. envelope 2 start value			*/
iAM2d	=  2		/* amp. envelope 2 decay speed			*/

/* ------------------------------------------------------------------------- */

ibtime	=  60/ibpm

p3	=  p3+irel+0.05

imkey	=  21
imvel	=  p5

icps	=  $MIDI2CPS(imkey)
iamp	=  (0.0039+imvel*imvel/16192)*ivol*16384
kamp	linseg 0, 0.02, 1 ,p3-(irel+0.06),1,irel,0,0.05,0

k_	line 0, 1, 1				/* calculate osc. frequency */
kcps	=  icps*(k_*(ifdec1/ibtime)+istrtf1)/(k_*(ifdec1/ibtime)+1)

knumh	=  sr/(2*kcps)				/* square oscillator */

a__	buzz sr/(10*3.14159265), kcps, knumh, 2560, 0
a___	buzz sr/(10*3.14159265), kcps, knumh, 2560, 0.5
a0	tone a__-a___, 10

a_	unirand 2				/* noise generator */
a0	=  a0+(a_-1)*insmix

a1	=  a0	/* save osc. output */

a0	butterhp a0,iHP1f*kcps			/* HP filter 1 */

a0	butterbp a0,iBP1f*kcps,iBP1b*kcps	/* BP filter 1 */

a_	butterhp a1,iHP2f*kcps			/* HP filter 2 and 3 */
a__	butterhp a1,iHP3f*kcps
a0	=  a0+a_*iHP2m+a__*iHP3m

a0	butterhp a0,iHPxf*icps				/* output HP filter */
a0	pareq a0,iHPxf*icps,iHPxr*1.4142,iHPxr,0

k_	port 1,ibtime/iAMxd,0				/* amp. envelope */
k__	=  sin(k_*3.14159265*2.0)
a0	=  a0*k__

k_	expseg 1,ibtime/iAM1d,0.5			/* amp. envelopes */
k__	expseg 1,ibtime/iAM2d,0.5
k_	=  (1-k_)*(1-iAM1s)+iAM1s
k__	=  (1-k__)*(1-iAM2s)+iAM2s
a0	=  a0*k_*k__

k_	port 0,ibtime/iLPd1,iLPf1			/* output LP filter */
k__	port 0,ibtime/iLPd2,iLPf2
a0	butterlp a0,kcps*(k_+k__)

a0	butterlp a0*iamp*kamp,sr*0.441

outs      a0*sqrt(ipan),a0*sqrt(1-ipan)

garvbsig  =         garvbsig+(a0*.005)
	;out a0, a0

	endin

/* ------------ instr 99: decoder and output instrument ------------ */

	instr 99

iamp	=  0.000001 * 0.000001 * 0.000001 * 0.000001

a0	rnd31 iamp, 0, 0	; low level noise to avoid denormals
a1	rnd31 iamp, 0, 0
a2	rnd31 iamp, 0, 0
a3	rnd31 iamp, 0, 0
a4	rnd31 iamp, 0, 0

	vincr a0, ga0 * p4	; get input from global variables
	vincr a1, ga1 * p4
	vincr a2, ga2 * p4
	vincr a3, ga3 * p4
	vincr a4, ga4 * p4

	clear ga0	; clear global vars
	clear ga1
	clear ga2
	clear ga3
	clear ga4

; decode to 2 chnls with phase shift

a1re, a1im	hilbert a1
a2re, a2im	hilbert a2
a3re, a3im	hilbert a3

aL	=  0.7071 * (a1re - a1im) + 0.5 * (a2re + a2im + a3re - a3im)
aR	=  0.7071 * (a1re + a1im) + 0.5 * (a2re - a2im - a3re - a3im)

	outs aL, aR

; mono output

	soundout a0, $SNDFL_MONO, 6

	endin

;**************************************************************************
; INSTR 101:        ELABORATE VOCAL INSTRUMENT: 76 P-FIELDS!
;		MALE & FEMALE VOICED, UNVOICED, MIXED, & CHORUSED DIPTHONGS
;
;You may use converted MIDI files since the orchestra ignores P4.
;Just cut/paste p6-p77 beneath each note-event in the score
;**************************************************************************

instr 101
	kvol chnget 	"fof1"
	kpchContr chnget 	"fof2"
	kformant chnget 	"fof3"
	kpchFromBass chnget 	"fof4"
	
	iamp = p4
	kpitch0 = p5+0.12*kpchContr + 0.01*kpchFromBass
	kpitch1 = kpitch0 + 0.07
	kpch = cpspch(kpitch0)
	kpch0 = cpspch(kpitch1)

	;icross = 0.01;
	
	;idur16 = idur/16
	;icross = idur16*0.05;
	;icrossPan = idur16*0.9;
	;Copy formant-table contents into variables (5 formants and 4 amplitudes)
			     ;index   tablenum	
/*			     
f2011 0 16 -2  360  750 2400 2675 2950 .05 .016 .018  .008	;11 oo (boot)
f2012 0 16 -2  325  700 2550 2850 3100 .065 .032 .035 .015	;12 O  (bow)
f2013 0 16 -2  415 1400 2200 2800 3300 .08 .06  .04  .01	 	;13 u  (foot)
f2014 0 16 -2  400 1050 2200 2650 3100 .13 .10  .11  .05  	;14 uh (but)
f2015 0 16 -2  609 1000 2450 2700 3240 .12 .05 .04   .005 	;15 ah (hot)
f2016 0 16 -2  300 1600 2150 2700 3100 .23 .25 .22   .10 	 	;16 er (bird)
f2017 0 16 -2  400 1700 2300 2900 3400 .30 .40 .25   .18		;17 e (bet)
f2018 0 16 -2  400 1700 2900 2700 3400 .09 .09 .05   .01		;18 iy (bee)

;Female: Vowel Formants
f2031 0 16 -2  280  650 2200 3450 4500 .15  .05  .04  .03		;31 oo (boot)
f2032 0 16 -2  400  840 2800 3250 4500 .20  .1   .13  .06 	;32 O  (bow)
f2033 0 16 -2  330 2000 2800 3650 5000 .16  .10  .07  .017 	;33 IY (bit) 
f2034 0 16 -2  500 1750 2450 3350 5000 .05  .04  .03  .02 	;34 E  (egg)
f2035 0 16 -2  650 1100 2860 3300 4500 .07  .05 .038  .027	;35 A  (egg)*/

ifoffr0aa = 609
ifoffr1aa = 1000
ifoffr2aa = 2450
ifoffr3aa = 2700
ifoffr4aa = 3240
ifofamp1aa = .12
ifofamp2aa = .05
ifofamp3aa = .04
ifofamp4aa = .005

ifoffr0oo = 360
ifoffr1oo = 750
ifoffr2oo = 2400
ifoffr3oo = 2675
ifoffr4oo = 2950
ifofamp1oo = .05
ifofamp2oo = .016
ifofamp3oo = .018
ifofamp4oo = .008

	;FOF OPCODES: VOICED FORMANTS FOR SOLO VOICE or 1st IN CHORUS
	;		  (any rate: const.	|                   |(must be given fixed val.|
	;         (ctrl., or audio )|(only &ctrl. rates)|   at initialization)
	;ar  fof  xamp  	   xfund  xform koct kband kris  kdur kdec  iolaps ifna ifnb 		;totdur [iphs] [ifmode]
	aA0  fof   iamp,  kpch,kformant*ifoffr0aa+(1-kformant)*ifoffr0oo, 0 ,  40,  .001, .08, .002,  120,    2001,   2002,   3600
	aA1  fof   iamp*(kformant*ifofamp1aa+(1-kformant)*ifofamp1oo), kpch,kformant*ifoffr1aa+(1-kformant)*ifoffr1oo,0,  40,  .001, .08, .002,  120,    2001,   2002,   3600
	aA2  fof   iamp*(kformant*ifofamp2aa+(1-kformant)*ifofamp2oo), kpch,kformant*ifoffr2aa+(1-kformant)*ifoffr2oo,0,  40,  .001, .08, .002,  120,    2001,   2002,   3600
	aA3  fof   iamp*(kformant*ifofamp3aa+(1-kformant)*ifofamp3oo), kpch,kformant*ifoffr3aa+(1-kformant)*ifoffr3oo,0,  40,  .001, .08, .002,  120,    2001,   2002,    3600
	aA4  fof   iamp*(kformant*ifofamp4aa+(1-kformant)*ifofamp4oo), kpch,kformant*ifoffr4aa+(1-kformant)*ifoffr4oo,0,  40,  .001, .08, .002,  120,    2001,   2002,    3600
	

	aB0  fof   iamp,  	kpch0,kformant*ifoffr0aa+(1-kformant)*ifoffr0oo, 0 ,  40,  .001, .08, .002,  120,    2001,   2002,   3600
	aB1  fof   iamp*(kformant*ifofamp1aa+(1-kformant)*ifofamp1oo), kpch0,kformant*ifoffr1aa+(1-kformant)*ifoffr1oo,0,  40,  .001, .08, .002,  120,    2001,   2002,   3600
	aB2  fof   iamp*(kformant*ifofamp2aa+(1-kformant)*ifofamp2oo), kpch0,kformant*ifoffr2aa+(1-kformant)*ifoffr2oo,0,  40,  .001, .08, .002,  120,    2001,   2002,   3600
	aB3  fof   iamp*(kformant*ifofamp3aa+(1-kformant)*ifofamp3oo), kpch0,kformant*ifoffr3aa+(1-kformant)*ifoffr3oo,0,  40,  .001, .08, .002,  120,    2001,   2002,    3600
	aB4  fof   iamp*(kformant*ifofamp4aa+(1-kformant)*ifofamp4oo), kpch0,kformant*ifoffr4aa+(1-kformant)*ifoffr4oo,0,  40,  .001, .08, .002,  120,    2001,   2002,    3600
	

	aA = kvol*(aA0+aA1+aA2+aA3+aA4)
	aB = kvol*(aB0+aB1+aB2+aB3+aB4)
	

		aDryL = aA ;kpan	
		aDryR = aB ;(1-kpan)

					
		outs  aDryL, aDryR 
		
		garvbsig  =         garvbsig+(aA+aB)*.005

endin


;===============================================
; INSTRUMENT 102 - a simple fm instrument, pulse
;===============================================
/*
          instr 102
ipan      		=       p5
iamp 			=  		6000
kIndexControl chnget 	"Pulse1"
kcar 		  chnget 	"Pulse2"
kmod 		  chnget 	"Pulse3"
kVolume 	  chnget 	"Pulse4"
kfreq		  chnget 	"Pulse5"
koct		  chnget 	"Pulse6"

kVolume       =  kVolume*iamp 

idurAttack	   =		 0.02
idurDecay 		= 		 0.02
idurSustain 	= 		p3 - (idurAttack+idurDecay)

kcps = p4*0.125*kfreq*koct

k1	linseg 0, idurAttack, 1,  idurSustain , 1, idurDecay, 0
a2  foscil k1*kVolume, kcps, kcar, kmod, kIndexControl, 1
    
          outs      a2*sqrt(ipan),a2*sqrt(1-ipan)
          

garvbsig  =         garvbsig+(a2*.08)

          endin
          
          

*/
;===============================================
; INSTRUMENT 103 - BASS INSTRUMENT
;===============================================
 opcode	moogladdersr,a,akk
	asig,kcf,kres	xin
		setksmps	1
	acf	interp	kcf
	kcf	downsamp	acf
	asig      moogladder   asig, kcf, kres
		xout	asig
endop

instr	103
kVolume chnget "Bass1"
kPitch chnget "Bass2"
kCfBase chnget "Bass3"
;kRes1 = 0.2+0.8*kRes1
iAmp  = p4	* 1.7
;iPitch = p5 ;6.02
iAcc = p5 *0.8

;kCfBase = 8.5 ;cutoff
kCfBase = 8.5+1*(1-kCfBase)
ikCfEnv = 3.5 ;env mod
ikDecay = 0.37 ;accent
ikRes1 = 0.5;resonance
ikTuning = 0; tuning
ikDist = 0.2 ;distortion
ikVol = 1 ;volume

	iTempo = 240*iAcc
	iPhFreq   =           iTempo/240	;FREQUENCY WITH WHICH TO REPEAT THE ENTIRE PHRASE
	iBtFreq   =           iTempo/15	;FREQUENCY OF EACH 1/16TH NOTE

	kporttime	linseg	0,0.001,0.01	;PORTAMENTO TIME RAMPS UP QUICKLY FROM ZERO TO A HELD VALUE

	kAcc	portk	iAcc, kporttime	;SCALE ACCENT USING ON-SCREEN KNOB AND SMOOTH CHANGES IN ACCENT VALUE TO PREVENT CLICKS
	
	kHoldDel  vdel_k       1, 1/(iBtFreq*2), 1	;OFFSET HOLD SO THAT ITS VALUE WILL OVERLAP THE NEXT NOTE
	kOct      portk        octpch(kPitch), 0.01*kHoldDel	;APPLY PORTAMENTO TO PITCH CHANGES - IF NOTE IS NOT HELD, NO PORTAMENTO WILL BE APPLIED
		
	;FILTER ENVELOPE
	kFiltRetrig	=	(1-kHoldDel)  ; + kOnTrig
	;CREATE A FILTER CUTOFF FREQUENCY ENVELOPE. FILTER ATTACK LEVEL IS A SUM OF BASE FREQUENCY, ENVELOPE DEPTH AND ACCENT
	kCfOct	  loopseg	1/3600, kFiltRetrig, 0, kCfBase+ikCfEnv+(kAcc*2), ikDecay, kCfBase, 3600-ikDecay, kCfBase
	kCfOct    limit        kCfOct, 4, 14	;LIMIT THE CUTOFF FREQUENCY TO BE WITHIN SENSIBLE LIMITS

	;AMPLITUDE ENVELOPE - SEPARATES NOTES THAT ARE NOT HELD
	katt	=	0.02 * (60/iTempo)
	kdec	=	0.02 * (60/iTempo)
	kAmpEnv   loopseg      iBtFreq, 0,    0,0,  katt,1,  (1/iBtFreq)-katt-kdec,1,  kdec,0	;SUSTAIN SEGMENT DURATION (AND THEREFORE ATTACK AND DECAY SEGMENT DURATIONS) ARE DEPENDENT UPON TEMPO
	kAmp	=	(kHoldDel==1?1:kAmpEnv)
	
	;AUDIO OSCILLATOR
	;A LABEL
	aSig      vco2         0.2, cpsoct(kOct)*semitone(ikTuning), 0 , 0.5	;GENERATE AUDIO USING VCO OSCILLATOR i(gkWaveform)*2

	;FILTER (CALLS UDO: A VERSION OF moogladder IN WHICH CUTOFF FREQUENCY IS MODULATED AT kr=sr) IN ORDER TO PREVENT QUANTISATION NOISE)
	kres	limit	ikRes1+(kAcc*0.4),0,0.98				;PREVENT EXCESSIVE RESONANCE THAT COULD RESULT FROM THE COMBINATION OF RESONANCE ;SETTING AND ACCENTING
	aFilt      moogladdersr   aSig, cpsoct(kCfOct), kres		;FILTER AUDIO
	aFilt	buthp	aFilt,50
	aSig	balance	aFilt,aSig
	; DISTORTION
	if ikDist==0 kgoto SKIP_DISTORTION 
	iSclLimit ftgentmp     0, 0, 1024, -16, 1, 1024,  -8, 0.01	;RESCALING CURVE FOR CLIP 'LIMIT' PARAMETER
	iSclGain  ftgentmp     0, 0, 1024, -16, 1, 1024,   4, 10	;RESCALING CURVE FOR GAIN COMPENSATION
	kLimit    table        ikDist, iSclLimit, 1			;READ LIMIT VALUE FROM RESCALING CURVE
	kGain     table        ikDist, iSclGain, 1			;READ GAIN VALUE FROM RESCALING CURVE
	kTrigDist changed      kLimit					;IF LIMIT VALUE CHANGES GENERATE A 'BANG'
	if kTrigDist=1 then						;IF A 'BANG' HAS BEEN GENERATED...
		reinit REINIT_CLIP					;BEGIN A REINITIALIZATION PASS FROM LABEL 'REINIT_CLIP'
	endif
	REINIT_CLIP:
	aSig      clip         aSig, 0, i(kLimit)			;CLIP DISTORT AUDIO SIGNAL
	rireturn							;
	aSig      =            aSig * kGain				;COMPENSATE FOR GAIN LOSS FROM 'CLIP' PROCESSING
	SKIP_DISTORTION:
	
	aAmp	interp	(kAmp*((kAcc*0.7)+1)*ikVol)		;COMBINE ALL FACTORS THAT CONTRIBUTE TO AMPLITUDE AND INTERPOLATE AND CREATE AN A-RATE VERSION OF THIS RESULT (TO PREVENT QUANTISATION NOISE)
	aSig	=	aSig * aAmp *iAmp *kVolume					;SCALE AUDIO USING AMPLITUDE CONTROL FUNCTION
	
	outs	aSig, aSig     					;AUDIO SENT TO OUTPUT, APPLY AMP. ENVELOPE, VOLUME CONTROL AND NOTE ON/OFF STATUS
	garvbsig  =         garvbsig+(aSig*.035)
endin
  
;-------------------------------------------------
; ********** THE ELECTRIC PRIEST ***" gliss "***
;-------------------------------------------------
          instr 104
kVolume chnget "Solo01"
k90 chnget "Solo02"
kform chnget "Solo03"
kformPitch = 30 + 50*kform
;k90 = k90*9
kPitch chnget "Bass2"
       
knote     =         cpspch(kPitch+k90*0.01)


 ;    line     120,p3,-120    offset                     ; GLISS: 1 CYCLE PER 4 BARS IF 0.4SEC =1 QUARTERNOTE
;iformPitch = 70

iamp = 10000
iamp1 = 4000
iamp2 = 1650
iamp3 = 1347
k21     = 456;  line      456,p3,1030  С‚РµРјР±СЂ

    ;linseg    0,p3*.9,0,p3*.1,1                  ; OCTAVIATION COEFFICIENT
a1        oscil     7,.15,3001                            ; RUBATO FOR VIBRATO
;a3        linen     a1,(p3-p3*.05),p3,.2               ; DELAY FOR VIBRATO
a2        oscil     a1,5,3001                             ; VIBRATO
;a5        linen     1250,p7,p3,(p3*.1)                 ; AMP ENVELOPE


 ; p6: MORPH-TIME,0=INSTANT AAH                         
;a5        linen     10000,p7,p3,(p3*.1)                ; AMP ENVELOPE
a11       fof       10000,knote+a2,k21*(kformPitch/100),0,200,.003,.017,.005,10,3001,3002,3600,0,1

;a31 =4000;      line      4000,p6,6845
;a32 =2471;      line      2471,p6,1370                                                
;a6        linen     a31,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a12       fof       iamp1,knote+a2,2471*(kformPitch/100),0,200,.003,.017,.005,20,3001,3002,3600,0,1

;a41  =2813;     line      2813,p6,3170
;a42  =1650;     line      1650,p6,1845                                                
;a7        linen     a42,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a13       fof       iamp2,knote+a2,2813*(kformPitch/100),0,200,.003,.017,.005,20,3001,3002,3600,0,1

;a71  =1347;     line      1347,p6,1726                       ; AMP LINE
;a72 =3839;      line      3839,p6,3797                       ; FORM LINE                                                 
;a8        linen     a71,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a14       fof       iamp3,knote+a2,3839*(kformPitch/100),0,200,.003,.017,.005,30,3001,3002,3600,0,1

;a51  =1;     line      1,p6,1250                                              
;a9        linen     a51,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a15       fof       iamp,knote+a2,4177*(kformPitch/100),0,200,.003,.017,.005,30,3001,3002,3600,0,1

;a61       line      1,p6,5833
;a10       linen     a51,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a16       fof       1,knote+a2,428*(kformPitch/100),0,200,.003,.017,.005,10,3001,3002,3600,0,1


a7        =         (a11+a12+a13+a14+a15+a16)*kVolume*0.5

          outs      a7,a7*0.2
          
garvbsig  =         garvbsig+a7*.1

          endin 
          
;-------------------------------------------------
; ********** THE ELECTRIC PRIEST ***" gliss "*** на октаву выше i 104 и справа
;-------------------------------------------------
          instr 109
kVolume chnget "Solo11"
k90 chnget "Solo12"
kform chnget "Solo13"
kformPitch = 30 + 50*kform
;k90 = k90*9


kPitch chnget "Bass2"
kPitch1 = kPitch+k90*0.01
knote     =   cpspch(kPitch1+0.12)
if(knote<580) kgoto label1
knote = knote*0.5
label1: 



 ;    line     120,p3,-120    offset                     ; GLISS: 1 CYCLE PER 4 BARS IF 0.4SEC =1 QUARTERNOTE
;iformPitch = 70

iamp = 10000
iamp1 = 4000
iamp2 = 1650
iamp3 = 1347
k21     = 456;  line      456,p3,1030  С‚РµРјР±СЂ

    ;linseg    0,p3*.9,0,p3*.1,1                  ; OCTAVIATION COEFFICIENT
a1        oscil     7,.15,3001                            ; RUBATO FOR VIBRATO
;a3        linen     a1,(p3-p3*.05),p3,.2               ; DELAY FOR VIBRATO
a2        oscil     a1,5,3001                             ; VIBRATO
;a5        linen     1250,p7,p3,(p3*.1)                 ; AMP ENVELOPE


 ; p6: MORPH-TIME,0=INSTANT AAH                         
;a5        linen     10000,p7,p3,(p3*.1)                ; AMP ENVELOPE
a11       fof       10000,knote+a2,k21*(kformPitch/100),0,200,.003,.017,.005,10,3001,3002,3600,0,1

;a31 =4000;      line      4000,p6,6845
;a32 =2471;      line      2471,p6,1370                                                
;a6        linen     a31,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a12       fof       iamp1,knote+a2,2471*(kformPitch/100),0,200,.003,.017,.005,20,3001,3002,3600,0,1

;a41  =2813;     line      2813,p6,3170
;a42  =1650;     line      1650,p6,1845                                                
;a7        linen     a42,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a13       fof       iamp2,knote+a2,2813*(kformPitch/100),0,200,.003,.017,.005,20,3001,3002,3600,0,1

;a71  =1347;     line      1347,p6,1726                       ; AMP LINE
;a72 =3839;      line      3839,p6,3797                       ; FORM LINE                                                 
;a8        linen     a71,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a14       fof       iamp3,knote+a2,3839*(kformPitch/100),0,200,.003,.017,.005,30,3001,3002,3600,0,1

;a51  =1;     line      1,p6,1250                                              
;a9        linen     a51,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a15       fof       iamp,knote+a2,4177*(kformPitch/100),0,200,.003,.017,.005,30,3001,3002,3600,0,1

;a61       line      1,p6,5833
;a10       linen     a51,p7,p3,(p3*.1)                  ; AMP ENVELOPE
a16       fof       1,knote+a2,428*(kformPitch/100),0,200,.003,.017,.005,10,3001,3002,3600,0,1


a7        =         (a11+a12+a13+a14+a15+a16)*kVolume*0.4

          outs      a7*0.2,a7
          
garvbsig  =         garvbsig+a7*.1

          endin

;-------------------------------------------------
; ********** vox(Dei)***
;-------------------------------------------------          
       instr 105; vox(Dei)
       
;idur = p3
kVolume chnget "Solo11"
kamp = kVolume*12000
k1 chnget "Solo12"
k1 = k1*7+1
;iamp = ampdb(p4)
;if1 = cpspch(p5)
kPitch chnget "Bass2"
kfr1 = cpspch(kPitch)
kFilter chnget "Solo13"

   ;kmod = .2;expseg .001,3.4,.001,1.25,.2,1.25,.005
   kf1 oscil kFilter*10,8,1051

;k1 line 1, p3, 4 ;linseg 1,  3.4, 3,   3,   3,  .39,  4, .4,3,.39,4,.4,3,.15,3.5,.15,3,.15,2.5,.15,3,8.5,2.8
 ;(control of form.freqency) 
;     xamp       xfund  xform     koct    kband  kris  kdur  kdec  iolaps  ifna ifnb itotdur iphs?  ifmode?
a2 fof kamp, kfr1, kfr1*k1+kf1     ,0.3,    1,   .003,  .02, .007,  30,   1051,   1052,   3600 ;, 0,    3
a3 fof kamp, kfr1, kfr1*k1+kf1     ,0.3,    2,   .005,  .02, .007,  30,   1053,   1052,   3600 ;,  0,    3
a1=a2+a3
alp butterlp a1,kfr1*k1+kf1
;a01 reverb2 a1,3.5,.65
a0=a1+alp*.5 ;+a01*.3

outs a0*0.2,a0*0.8
garvbsig  =         garvbsig+a0*.1

  endin 
  
  ;-------------------------------------------------
; ********** Ah***
;-------------------------------------------------  
  
                 instr     106

kPitchControl chnget "Bass2"
kcoeff chnget "Solo12"
kpitch = cpspch(kPitchControl+kcoeff*0.01+0.12)

kvibpnt chnget "Solo13"
kvibpnt          = kvibpnt*1.9+0.1; control y 0.1 -   2
 ;PERCENT VIBRATO (OF FUNDAMENTAL FREQ.)
 
kVolume chnget "Solo11"
kenv adsr .05, 3600, 1, 0
kamp         = 32768*kVolume*kenv ;control


kvibamt        = 1

kvib           oscili    kvibamt, 4 + (kvibamt * 2), 1061

kf0            =         kpitch + (kvib * kvibpnt / 100 * kpitch)
knharm         =         sr/2/kf0                      ;NUMBER OF HARMONICS FOR DRIVING F0

abuz           buzz      kenv, kf0, knharm, 1061
avoc           tone      abuz, kf0 * 1.5               ;ROLLOFF FILTER 12DB/OCTAVE

kbandw         =         200 + (kenv * 200)            ;OPEN UP BANDWIDTH ALONG WITH AMP

aform1         reson     avoc, 609, kbandw, 1
aform2         reson     avoc, 1000, 300, 1
aform3         reson     avoc, 2450, 300, 1
aform4         reson     avoc, 2700, 250, 1
aform5         reson     avoc, 3240, 250, 1

afout          =         aform1 + (aform2 / 2) + (aform3 / 4) + (aform4 / 3.5) + (aform5 / 16)
afil           =         afout / 5                     ; FIVE FILTERS, SO AVERAGE
abal           balance   afil, abuz                    ; THEN BALANCE WITH ORIGINAL DRIVE

;arevb          reverb    abal, .5
;arev           =         arevb / 2.3

aout           =         abal * kamp *3 ;arev +
               outs      aout*0.2, aout*0.8
               garvbsig  =         garvbsig+aout*.1
               endin 
               
;----------------------------------
;			saw ping	  
;----------------------------------

			instr 	107
			
kPitchControl chnget "Bass2"
kcoeff chnget "Solo12"
kfreq = cpspch(kPitchControl+kcoeff*0.01+0.24)

kVolume chnget "Solo11"
kenv adsr .05, 3600, 1, 0
kamp         = 60000*kVolume*kenv ;control			

kcontr chnget "Solo13"
kfdepth		= 5000;		kcontr*9000 +1000	;control y 	1000,-10000
kfreqline		=		kcontr*5 +5	;control y 	1000,-10000

kffreq		=		5+kcontr*12	
ibw			=		100

kpan		=		0.2 ;line	1, p3, 0

kfiltmod	oscil	kfdepth, kffreq, 1072
abuzz1		buzz	kamp / 4, kfreq, 100, 1071
abuzz2		buzz	kamp / 4, kfreq, 10, 1071
abuzz3		buzz	kamp / 4 , kfreq / .99, 100, 1071
abuzz4		buzz	kamp / 4, kfreq / .99, 10, 1071
amix		=		abuzz1 + abuzz2 + abuzz3 +abuzz4
afilt1	=amix;	butterlp	amix, kfiltmod, ibw

aampmod 	oscil	0.5, kfreqline, 1083
afilt1 = afilt1*(aampmod+0.5)

			outs	afilt1 * kpan , afilt1 * (1 - kpan)
		 garvbsig  =         garvbsig+afilt1*.02	
			endin

;----------------------------------
;	; Table Based PWM Rezzy Synth	  
;----------------------------------			
instr 108   ; Table Based PWM Rezzy Synth

kVolume chnget "Solo11"
kenv adsr .05, 3600, 1, 0
kamp     = 10000;*kVolume*kenv ;control
;iamp   =  10000; volume control

kPitchControl chnget "Bass2"
kcoeff chnget "Solo12"
kfreq = cpspch(9.01);(kPitchControl+kcoeff*0.01+0.36)
;ifqc   = cpspch(8.09) ;control x

irez   = 20 

kcontr =1;chnget "Solo13"
kfrform1 = 280 +370*kcontr
kfrform2 = 650 +450*kcontr
kfrform3 = 2200+660*kcontr
kfrform4 = 3450-150*kcontr
ifrform5 = 4500

iampform1 = 1000
kampform2 = 150 - 80*kcontr
iampform3 = 50
kampform4 = 40 -2*kcontr
kampform5 = 30 - 3*kcontr
 

;Frequency of amp change
kfreqline  = 5*kcontr +5;line 5, p3, 10 ;control y
; Frequency Sweep
kfco = 5;linseg 5, .5*p3, 200, .5*p3, 5 ;control y

; Amplitude envelope
kaenv adsr .05, 3600, 1, 0
;kaenv  linseg 0, .01, 1, p3-.02, 1, .01, 0
  

; This relationship attempts to separate Freq from Res.
aa1 = 100/irez/sqrt(kfco)-1
aa2 = 1000/kfco

; Initialize Yn-1 & Yn-2 to zero
aynm1 init 0
aynm2 init 0

; PWM Oscillator
  asine   oscil 1.5,        kfreq/440,     1081
  asquare oscil kfreq*asine, kfreq,         1082
  axn     oscil kamp,       kfreq+asquare, 1082

; Replace the differential eq. with a difference eq.
  ayn = ((aa1+2*aa2)*aynm1-aa2*aynm2+axn)/(1+aa1+aa2)
  aynm2 = aynm1
  aynm1 = ayn

; Amp envelope and output
  aout = ayn * kaenv
 

aform1         reson     aout, kfrform1, iampform1, 1
aform2         reson     aout, kfrform2, kampform2, 1
aform3         reson     aout, kfrform3, iampform3, 1
aform4         reson     aout, kfrform4, kampform4, 1
aform5         reson     aout, ifrform5, kampform5, 1


/*Female: Vowel Formants
f2031 0 16 -2  280  650 2200 3450 4500 .15  .05  .04  .03		;31 oo (boot)
f2032 0 16 -2  400  840 2800 3250 4500 .20  .1   .13  .06 	;32 O  (bow)
f2033 0 16 -2  330 2000 2800 3650 5000 .16  .10  .07  .017 	;33 IY (bit) 
f2034 0 16 -2  500 1750 2450 3350 5000 .05  .04  .03  .02 	;34 E  (egg)
f2035 0 16 -2  650 1100 2860 3300 4500 .07  .05 .038  .027	;35 A  (egg)*/

afout          =         (aform1 + aform2   + aform3  + aform4 + aform5 )/5
  aout           balance   afout, aout
  
	aampmod 	oscil	0.5, kfreqline, 1083
	
    aout = aout*(aampmod+0.5)
              
 	outs aout*0.2, aout*0.8
 
  garvbsig  =         garvbsig+aout*.02
endin	
;===================
; GLOBAL REVERB
;===================

          instr 199

a1        reverb2   garvbsig, p4, p5
          outs      a1,a1

garvbsig  =         0

          endin

;====================
;
; GLOBAL DELAY
;
;====================

          instr 198            ; THIS DELAY IS IN PARALLEL CONFIG
                                             
a1        delay     gasig,p4                 ; DELAY=1.25
a2        delay     gasig,p4*2               ; DELAY=2.50
          outs      a1,a2

gasig     =         0

          endin
</CsInstruments>
<CsScore>
f0 3600

f1000 0 512 7 0 50 1 50 .5 300 .5 112 0                   ;ADSR
f1001 0 2048 10 1   
f1002 0 2048 10 1 0.5
f1003 0 2048 10 1 0.5 0.5 
f1004 0 2048 10 1 0.5 0.5 0.5 
f1005 0 2048 10 1 0.5 0.5 0.5 0.5  
f1006 0 2048 10 1 0.5 0.5 0.5 0.5 0.5
f1007 0 2048 10 1 0.5 0.5 0.5 0.5 0.5 0.5 
f1008 0 2048 10 1 0.5 0.5 0.5 0.5 0.5 0.5 ;i1
; ============ drums.sco - written by Istvan Varga, 2002 ============

; ---- tempo ----

;t 0 132

; -- instr 1: render tables for cymbal instruments --

; p3 : note length (should be 0)
; p4 : ftable number
; p5 : number of partials
; p6 : amp. scale
; p7 : transpose (in semitones)
; p8 : random seed (1 to 2^31 - 2)
; p9 : amplitude distribution

; ---- generate cymbal tables ----

i 1 0 0 101 600 1 0 114 3		; crash 2
i 1 0 0 102 600 1 0 4 6			; hihat
i 1 0 0 103 600 1 0 213 3		; crash 1
;i 1 0 0 104 600 1 0 427 4		; crash 3 (not used)
f 99 0 16 -2	0.3	7500	0	1	10500	0.2	\
		0.3	14000	0.4	1	18000	0.8
f 105 0 524288 -34 99 4 1 -4		; tambourine
i 1 0 0 106 600 1 0 193 6		; hihat 2
i 1 0 0 107 600 1 2 19 4		; ride
;i 1 0 0 108 600 1 0 7 4			; ride 2 (not used)

; ---- misc. tables ----

; square wave

#include "fgen_h.sco"

f 301 0 16384 7 1 8192 1 0 -1 8192 -1
$FGEN128(300'4096'301'1)

; sawtooth wave

f 501 0 16384 7 1 16384 -1
$FGEN128(500'4096'501'1)

; sine

f 700 0 4096 10 1

; window for cymbal instruments

f 100 0 16385 5 1 16385 0.01

; ---- include room parameters ----

#include "room.sco"

; ================ instrument definitions ================

/* ---- crash cymbal 1 ---- */

f 10 0 32 -2	900	; amplitude scale
		0.015	; delay
		0.1	; release time
		-1	; X
		1.87	; Y
		0	; Z
		103	; input table
		100	; window table
		0.225	; start grain duration in seconds
		0.10	; grain druaton envelope half-time
		0.1	; end grain duration
		40	; number of overlaps
		10000	; EQ start frequency
		1	; EQ frequency envelope half-time
		10000	; EQ end frequency
		1	; EQ start level (Q is level * 0.7071)
		0.14	; EQ level envelope half-time
		4	; EQ end level
		500	; highpass frequency
		20000	; lowpass frequency
		0.16	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.001	; delay time for chorus effect
		1	; non-delayed signal level
		0	; delayed signal level

/* ---- crash cymbal 2 ---- */

f 11 0 32 -2	900	; amplitude scale
		0.015	; delay
		0.1	; release time
		0.5	; X
		1.9	; Y
		0	; Z
		101	; input table
		100	; window table
		0.225	; start grain duration in seconds
		0.10	; grain druaton envelope half-time
		0.1	; end grain duration
		40	; number of overlaps
		11000	; EQ start frequency
		1	; EQ frequency envelope half-time
		11000	; EQ end frequency
		1	; EQ start level (Q is level * 0.7071)
		0.16	; EQ level envelope half-time
		3	; EQ end level
		500	; highpass frequency
		20000	; lowpass frequency
		0.18	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.001	; delay time for chorus effect
		1	; non-delayed signal level
		0	; delayed signal level

/* ---- reverse cymbal ---- */

f 14 0 32 -2	400	; amplitude scale
		0.016	; delay
		0.03	; release time
		-1.5	; X
		-1.7	; Y
		0	; Z
		103	; input table
		100	; window table
		0.1	; start grain duration in seconds
		0.2	; grain druaton envelope half-time
		0.1	; end grain duration
		50	; number of overlaps
		10000	; EQ start frequency
		1	; EQ frequency envelope half-time
		10000	; EQ end frequency
		2	; EQ start level (Q is level * 0.7071)
		0.2	; EQ level envelope half-time
		2	; EQ end level
		500	; highpass frequency
		18000	; lowpass frequency
		0.2	; decay env. half-time (n.a. in reverse mode)
		0	; reverse cymbal mode (0: on, 1: off)
		0.001	; delay time for chorus effect
		1	; non-delayed signal level
		0	; delayed signal level

/* ---- open hi-hat ---- */

f 13 0 32 -2	700	; amplitude scale
		0.012	; delay
		0.02	; release time
		1.5	; X
		1.5	; Y
		0	; Z
		102	; input table
		100	; window table
		0.001	; start grain duration in seconds
		0.05	; grain druaton envelope half-time
		0.1	; end grain duration
		50	; number of overlaps
		10000	; EQ start frequency
		1	; EQ frequency envelope half-time
		10000	; EQ end frequency
		2	; EQ start level (Q is level * 0.7071)
		1	; EQ level envelope half-time
		2	; EQ end level
		1000	; highpass frequency
		22000	; lowpass frequency
		0.1	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.015	; delay time for chorus effect
		0.2	; non-delayed signal level
		1	; delayed signal level

/* ---- closed hi-hat ---- */

f 12 0 32 -2	700	; amplitude scale
		0.0035	; delay
		0.02	; release time
		2.0	; X
		0.7	; Y
		0	; Z
		102	; input table
		100	; window table
		0.0001	; start grain duration in seconds
		0.01	; grain druaton envelope half-time
		0.1	; end grain duration
		50	; number of overlaps
		10000	; EQ start frequency
		1	; EQ frequency envelope half-time
		10000	; EQ end frequency
		2	; EQ start level (Q is level * 0.7071)
		1	; EQ level envelope half-time
		2	; EQ end level
		500	; highpass frequency
		22000	; lowpass frequency
		0.02	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.015	; delay time for chorus effect
		0	; non-delayed signal level
		1	; delayed signal level

/* ---- tambourine ---- */

f 15 0 32 -2	6500	; amplitude scale
		0.018	; delay
		0.02	; release time
		1.75	; X
		-1.2	; Y
		0	; Z
		105	; input table
		100	; window table
		0.002	; start grain duration in seconds
		0.01	; grain druaton envelope half-time
		0.03	; end grain duration
		20	; number of overlaps
		1000	; EQ start frequency
		1	; EQ frequency envelope half-time
		1000	; EQ end frequency
		1	; EQ start level (Q is level * 0.7071)
		1	; EQ level envelope half-time
		1	; EQ end level
		100	; highpass frequency
		22000	; lowpass frequency
		0.03	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.001	; delay time for chorus effect
		1	; non-delayed signal level
		0	; delayed signal level

/* ---- hi-hat 2 ---- */

f 16 0 32 -2	400	; amplitude scale
		0.008	; delay
		0.02	; release time
		-1.75	; X
		1.2	; Y
		0	; Z
		106	; input table
		100	; window table
		0.08	; start grain duration in seconds
		0.05	; grain druaton envelope half-time
		0.08	; end grain duration
		50	; number of overlaps
		8000	; EQ start frequency
		1	; EQ frequency envelope half-time
		8000	; EQ end frequency
		2	; EQ start level (Q is level * 0.7071)
		1	; EQ level envelope half-time
		2	; EQ end level
		1000	; highpass frequency
		14000	; lowpass frequency
		0.25	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.015	; delay time for chorus effect
		0.3	; non-delayed signal level
		1	; delayed signal level

/* ---- ride cymbal ---- */

f 17 0 32 -2	450	; amplitude scale
		0.02	; delay
		0.05	; release time
		-1.2	; X
		-1.75	; Y
		0	; Z
		107	; input table
		100	; window table
		0.0005	; start grain duration in seconds
		0.01	; grain druaton envelope half-time
		0.4	; end grain duration
		50	; number of overlaps
		12000	; EQ start frequency
		1	; EQ frequency envelope half-time
		12000	; EQ end frequency
		2	; EQ start level (Q is level * 0.7071)
		1	; EQ level envelope half-time
		2	; EQ end level
		1000	; highpass frequency
		22000	; lowpass frequency
		0.1	; decay env. half-time (n.a. in reverse mode)
		1	; reverse cymbal mode (0: on, 1: off)
		0.001	; delay time for chorus effect
		1	; non-delayed signal level
		0	; delayed signal level

/* ---- bass drum ---- */

f 20 0 64 -2	0.6	; volume
		0.020	; delay (in seconds)
		0.03	; release time (sec.)
		0	; X
		1	; Y
		0	; Z
		140	; tempo
		25	; base frequency (MIDI note number) изначально 31
		5.3333	; oscillator start frequency / base frequency
		0.0714	; oscillator freq. envelope half-time in beats
		0.5	; bandpass 1 bandwidth / oscillator frequency
		0.0625	; highpass 1 freq. / oscillator frequency
		0.5	; "allpass" 1 start freq. / oscillator frq.
		0.125	; "allpass" 1 envelope half-time in beats
		1.0	; "allpass" 1 end frq. / oscillator frequency
		8	; highpass 2 frequency / base frequency
		-3	; highpass 2 output gain
		0.5	; highpass 3 freq. / base frequency
		-0.4	; highpass 3 output gain
		1.5	; output highpass frequency / base frequency
		2	; output highpass resonance
		16	; output lowpass start freq 1 / oscillator frq.
		0.01	; output lowpass frequency 1 half-time in beats
		16	; output lowpass start freq 2 / oscillator frq.
		0.08	; output lowpass frequency 2 half-time in beats
		7040	; noise bandpass start frequency in Hz
		7040	; noise bandpass end frequency in Hz
		2	; noise bandpass bandwidth / frequency
		3520	; noise lowpass start frequency (Hz)
		55	; noise lowpass end frequency (Hz)
		0.0833	; noise filter envelope half-time in beats
		0.01	; noise attack time (in seconds)
		0.3333	; noise decay half-time (in beats)
		0.5	; noise mix

/* ---- TR-808 bass drum ---- */

f 25 0 16 -2	30000			/* amplitude scale	     */
		0.0115			/* delay		     */
		0.08			/* release time		     */
		0	1	0	/* X, Y, Z coordinates	     */
		32			/* base freq. (MIDI note)    */
		4			/* start freq. / base frq.   */
		0.007			/* frq. envelope half-time   */
		0.25			/* start phase (0..1)	     */
		3000			/* lowpass filter frequency  */
		0.10			/* decay half-time	     */

/* ---- hand clap ---- */

f 30 0 32 -2	550000000		/* amplitude scale	     */
		0.010			/* delay		     */
		0.02			/* release time		     */
		-0.5	2	0	/* X, Y, Z coordinates	     */
		1046.5			/* bandpass frequency	     */
		4186	0.03	261.63	/* bandwidth envelope start, */
					/* half-time, and end value  */
		0.011	0.023	0.031	/* delay 2, 3, and 4	     */
		0.0167	0.0167	0.0167	/* decay 1, 2, and 3	     */
		0.5			/* decay 4		     */

/* ---- TR-808 cowbell ---- */

f 35 0 16 -2	10000			/* amplitude scale	     */
		0.018			/* delay		     */
		0.05			/* release time		     */
		1.3	-1.5	0	/* X, Y, Z coordinates	     */
		73	80		/* osc 1, 2 freq (MIDI note) */
		20000			/* lowpass filter start frq. */
		0.025			/* filter envelope half-time */
		4000			/* lowpass filter end freq.  */
		0.002			/* attack time		     */
		0.03	0.3		/* decay time 1, level 1     */
		0.05			/* decay 2 half-time	     */
		4			/* resonance at osc 2 freq.  */

/* ---- TR-808 hi-hat (open) ---- */

; oscillator frequencies taken from Steven Cook's 808HiHat.orc

f 40 0 32 -2	20000			/* amplitude scale	     */
		0.02			/* delay		     */
		0.04			/* release time		     */
		-0.7	-2	0	/* X, Y, Z coordinates	     */
		73			/* base freq. (MIDI note)    */
		1.4471	1.6170		/* osc 2, 3 freq. / base frq */
		1.9265	2.5028	2.6637	/* osc 4, 5, 6 frq / base f. */
		0.25			/* distort start (see orc)   */
		1			/* distortion env. half-time */
		0.25			/* distort end		     */
		5400	1.0		/* highpass freq, resonance  */
		0.0005			/* attack time		     */
		0.2	0.5		/* decay time 1, level 1     */
		0.2			/* decay 2 half-time	     */

/* ---- TR-808 hi-hat (closed) ---- */

f 41 0 32 -2	20000			/* amplitude scale	     */
		0.02			/* delay		     */
		0.04			/* release time		     */
		-1.87	-1	0	/* X, Y, Z coordinates	     */
		73			/* base freq. (MIDI note)    */
		1.4471	1.6170		/* osc 2, 3 freq. / base frq */
		1.9265	2.5028	2.6637	/* osc 4, 5, 6 frq / base f. */
		0.25			/* distort start (see orc)   */
		1			/* distortion env. half-time */
		0.25			/* distort end		     */
		5400	1.0		/* highpass freq, resonance  */
		0.0005			/* attack time		     */
		0.025	0.5		/* decay time 1, level 1     */
		0.025			/* decay 2 half-time	     */

/* ---- TR-808 cymbal ---- */

f 42 0 32 -2	22000			/* amplitude scale	     */
		0.018			/* delay		     */
		0.1			/* release time		     */
		0.7	2.5	0	/* X, Y, Z coordinates	     */
		73			/* base freq. (MIDI note)    */
		1.4471	1.6170		/* osc 2, 3 freq. / base frq */
		1.9265	2.5028	2.6637	/* osc 4, 5, 6 frq / base f. */
		1.0			/* distort start (see orc)   */
		0.2			/* distortion env. half-time */
		0.0625			/* distort end		     */
		5400	0.7071		/* highpass freq, resonance  */
		0.0005			/* attack time		     */
		0.04	0.5		/* decay time 1, level 1     */
		0.4			/* decay 2 half-time	     */

/* ---- TR-909 snare drum 1 ---- */

f 50 0 32 -2	20000			/* amplitude scale	     */
		0.02			/* delay		     */
		0.04			/* release time		     */
		-0.5	1.25	0	/* X, Y, Z coordinates	     */
		49			/* base freq. (MIDI note)    */
		3			/* start freq. / base frq.   */
		0.005			/* frequency env. half-time  */
		0.5	0.005	0.2	/* FM depth start, envelope  */
					/*   half-time, end	     */
		1.4983			/* osc 2 frq. / osc 1 frq.   */
		1.0	0.01	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		2500	10000		/* noise BP freq., bandwidth */
		0	0.01	0.7	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.04			/* decay envelope half-time  */

/* ---- TR-909 snare drum 2 ---- */

f 51 0 32 -2	20000			/* amplitude scale	     */
		0.02			/* delay		     */
		0.04			/* release time		     */
		1	0.25	0	/* X, Y, Z coordinates	     */
		52			/* base freq. (MIDI note)    */
		2			/* start freq. / base frq.   */
		0.005			/* frequency env. half-time  */
		1.0	0.002	0	/* FM depth start, envelope  */
					/*   half-time, end	     */
		1.4983			/* osc 2 frq. / osc 1 frq.   */
		1	0.02	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		5000	7500		/* noise BP freq., bandwidth */
		0	0.008	0.35	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.03			/* decay envelope half-time  */

/* ---- TR-909 snare drum 3 ---- */

f 52 0 32 -2	17000			/* amplitude scale	     */
		0.02			/* delay		     */
		0.04			/* release time		     */
		0.75	1	0	/* X, Y, Z coordinates	     */
		52			/* base freq. (MIDI note)    */
		2.0			/* start freq. / base frq.   */
		0.005			/* frequency env. half-time  */
		0.2	0.01	0	/* FM depth start, envelope  */
					/*   half-time, end	     */
		1.4983			/* osc 2 frq. / osc 1 frq.   */
		1	0.04	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		5000	7500		/* noise BP freq., bandwidth */
		0	0.005	1	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.03			/* decay envelope half-time  */

/* ---- TR-909 snare drum 4 ---- */

f 53 0 32 -2	15000			/* amplitude scale	     */
		0.02			/* delay		     */
		0.04			/* release time		     */
		-0.75	0.75	0	/* X, Y, Z coordinates	     */
		56			/* base freq. (MIDI note)    */
		2			/* start freq. / base frq.   */
		0.0015			/* frequency env. half-time  */
		2.0	0.001	0	/* FM depth start, envelope  */
					/*   half-time, end	     */
		1.4983			/* osc 2 frq. / osc 1 frq.   */
		1	0.02	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		5000	7500		/* noise BP freq., bandwidth */
		0	0.005	1	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.03			/* decay envelope half-time  */

/* ---- hi tom ---- */

f 57 0 32 -2	25000			/* amplitude scale	     */
		0.018			/* delay		     */
		0.04			/* release time		     */
		2	-1	0	/* X, Y, Z coordinates	     */
		49			/* base freq. (MIDI note)    */
		1.3333			/* start freq. / base frq.   */
		0.135			/* frequency env. half-time  */
		2.0			/* osc 2 frq. / osc 1 frq.   */
		1	0.01	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		0			/* osc 1 and 2 phase (0 - 1) */
		0.083			/* invert 1 delay * base frq */
		0.135			/* invert 2 delay * base frq */
		10000			/* lowpass frequency	     */
		208	208		/* noise BP freq., bandwidth */
		1			/* noise comb freq / base f. */
		0.4			/* noise comb feedback	     */
		6000			/* noise lowpass frequency   */
		5	0.08	0	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.18			/* decay envelope half-time  */

/* ---- mid tom ---- */

f 58 0 32 -2	25000			/* amplitude scale	     */
		0.018			/* delay		     */
		0.04			/* release time		     */
		1	-2	0	/* X, Y, Z coordinates	     */
		44			/* base freq. (MIDI note)    */
		1.3333			/* start freq. / base frq.   */
		0.135			/* frequency env. half-time  */
		2.0			/* osc 2 frq. / osc 1 frq.   */
		1	0.01	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		0			/* osc 1 and 2 phase (0 - 1) */
		0.083			/* invert 1 delay * base frq */
		0.135			/* invert 2 delay * base frq */
		10000			/* lowpass frequency	     */
		208	208		/* noise BP freq., bandwidth */
		1			/* noise comb freq / base f. */
		0.4			/* noise comb feedback	     */
		6000			/* noise lowpass frequency   */
		5	0.08	0	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.18			/* decay envelope half-time  */

/* ---- low tom ---- */

f 59 0 32 -2	25000			/* amplitude scale	     */
		0.018			/* delay		     */
		0.04			/* release time		     */
		-2	-1	0	/* X, Y, Z coordinates	     */
		37			/* base freq. (MIDI note)    */
		1.3333			/* start freq. / base frq.   */
		0.135			/* frequency env. half-time  */
		2.0			/* osc 2 frq. / osc 1 frq.   */
		1	0.01	0	/* osc 2 amp. start, env.    */
					/*   half-time, end	     */
		0			/* osc 1 and 2 phase (0 - 1) */
		0.083			/* invert 1 delay * base frq */
		0.135			/* invert 2 delay * base frq */
		10000			/* lowpass frequency	     */
		208	208		/* noise BP freq., bandwidth */
		1			/* noise comb freq / base f. */
		0.4			/* noise comb feedback	     */
		6000			/* noise lowpass frequency   */
		5	0.08	0	/* noise amp. start, env.    */
					/*   half-time, end	     */
		0.18			/* decay envelope half-time  */

/* ---- rim shot ---- */

f 56 0 32 -2	10000			/* amplitude scale	     */
		0.0195			/* delay		     */
		0.04			/* release time		     */
		-1.5	0.5	0	/* X, Y, Z coordinates	     */
		56			/* base freq. (MIDI note)    */
		2			/* start freq. / base frq.   */
		0.0025			/* frequency env. half-time  */
		3	0.02	0	/* FM depth start, envelope  */
					/*   half-time, end	     */
		0.1	2000		/* noise amp., lowpass frq.  */
		4	0.006	0	/* amplitude (before	     */
					/*   distortion) start, env. */
					/*   half-time, end	     */
		-4	4000	0.0002	/* highpass filtered signal  */
					/*   (after distortion) amp, */
					/*   cutoff frequency, delay */
		20000	0.009	100	/* output lowpass start frq, */
					/*   env. half-time, end frq */

; ======== list of available instruments ========

; p-fields for all instruments:
;
;   p2: start time
;   p3: duration
;   p5: velocity
;
; instruments are selected by p1 and p4:
;
;   +------------------+------+------+
;   |    instrument    |  p1  |  p4  |
;   +------------------+------+------+
;   | crash cymbal 1   |  10  |  10  |
;   +------------------+------+------+
;   | crash cymbal 2   |  10  |  11  |
;   +------------------+------+------+
;   | reverse cymbal   |  10  |  12  |
;   +------------------+------+------+
;   | open hi-hat      |  10  |  13  |
;   +------------------+------+------+
;   | closed hi-hat    |  10  |  14  |
;   +------------------+------+------+
;   | tambourine       |  10  |  15  |
;   +------------------+------+------+
;   | open hi-hat 2    |  10  |  16  |
;   +------------------+------+------+
;   | ride cymbal      |  10  |  17  |
;   +------------------+------+------+
;   | bass drum        |  20  |  20  |
;   +------------------+------+------+
;   | TR-808 bass drum |  21  |  25  |
;   +------------------+------+------+
;   | hand clap        |  30  |  30  |
;   +------------------+------+------+
;   | TR-808 cowbell   |  31  |  35  |
;   +------------------+------+------+
;   | TR-808 open      |  40  |  40  |
;   | hi-hat           |      |      |
;   +------------------+------+------+
;   | TR-808 closed    |  40  |  41  |
;   | hi-hat           |      |      |
;   +------------------+------+------+
;   | TR-808 cymbal    |  40  |  42  |
;   +------------------+------+------+
;   | TR-909 snare 1   |  50  |  50  |
;   +------------------+------+------+
;   | TR-909 snare 2   |  50  |  51  |
;   +------------------+------+------+
;   | TR-909 snare 3   |  50  |  52  |
;   +------------------+------+------+
;   | TR-909 snare 4   |  50  |  53  |
;   +------------------+------+------+
;   | high tom         |  51  |  57  |
;   +------------------+------+------+
;   | mid tom          |  51  |  58  |
;   +------------------+------+------+
;   | low tom          |  51  |  59  |
;   +------------------+------+------+
;   | rim shot         |  52  |  56  |
;   +------------------+------+------+

; ---------------------------------------------------------------------

;#include "score.sco"

; ---------------------------------------------------------------------
;fof
;tabel   creation	table type/	table	
;number  time       size  GEN		values.....
f2001  	0  		4096  10  	1
f2002   	0    	1024  19  	.5  .5  270  .5

;Male: Vowel Formants / direct-indexed tables (freqs. 0-4, amps. 6-9)
f2011 0 16 -2  360  750 2400 2675 2950 .05 .016 .018  .008	;11 oo (boot)
f2012 0 16 -2  325  700 2550 2850 3100 .065 .032 .035 .015	;12 O  (bow)
f2013 0 16 -2  415 1400 2200 2800 3300 .08 .06  .04  .01	 	;13 u  (foot)
f2014 0 16 -2  400 1050 2200 2650 3100 .13 .10  .11  .05  	;14 uh (but)
f2015 0 16 -2  609 1000 2450 2700 3240 .12 .05 .04   .005 	;15 ah (hot)
f2016 0 16 -2  300 1600 2150 2700 3100 .23 .25 .22   .10 	 	;16 er (bird)
f2017 0 16 -2  400 1700 2300 2900 3400 .30 .40 .25   .18		;17 e (bet)
f2018 0 16 -2  400 1700 2900 2700 3400 .09 .09 .05   .01		;18 iy (bee)

;Female: Vowel Formants
f2031 0 16 -2  280  650 2200 3450 4500 .15  .05  .04  .03		;31 oo (boot)
f2032 0 16 -2  400  840 2800 3250 4500 .20  .1   .13  .06 	;32 O  (bow)
f2033 0 16 -2  330 2000 2800 3650 5000 .16  .10  .07  .017 	;33 IY (bit) 
f2034 0 16 -2  500 1750 2450 3350 5000 .05  .04  .03  .02 	;34 E  (egg)
f2035 0 16 -2  650 1100 2860 3300 4500 .07  .05 .038  .027	;35 A  (egg)
;~fof

;i104
f3001  0   4096 10  1                                          ; FOF
f3002  0   1024 19  .5  .5  270  .5                            ; FOF
;~i104

;i105
f1051 0 65536 10 3 
f1052 0 4096 7 0 4096 20
f1053 0 4096 11 2
;~i105

;i106
f1061 0 8193 10 1
;i106

;i107
f1071 	0 	2048 	10 	1
f1072	0	4096	10	1 0 .333 0 .2 0 .142 0 .111 0 .09 0 .076 0 .066 0 
;~i107

;i108
;**********************************************************
; f1=Sine, f2=Square, f3=Sawtooth, f4=Triangle, f5=Sqare2 
;**********************************************************
f1081  0  1024  10   1
f1082  0   256   7  -1 128 -1   0  1 128  1
f1083	0	4096	10	1 0 .333 0 .2 0 .142 0 .111 0 .09 0 .076 0 .066 0 
;~i108

 


f1 0 65536 10 1
f 2560 0 262144 10 1
; output instrument (p6: volume)
i199  0    3600  6    .2
i99 0 3600 0.3
;i60	 0.000	2.000 35 0.8	
;i61	1	0.4000	1.1	127 0
;i61	2	0.4000	1.3	127 0
;i61	3	0.4000	1.5	127 0
;i61	4	0.4000	1.7	127 0
;i51 	1 	0.15 	1000 127 	1
;i51 	3 	0.15 	4000 127 	1
;i51 	5	0.15 	6000 127 	1
;i52 1 0.3 3 127 0
;i52 2 0.3 3 127 0
;i52 3 0.3 3 127 0
;i52 4 0.3 3 127 0
;i52 5 0.3 3 127 0
;i10 1 0.3 22000 127 1
;i10 2 0.3 11000 127 1
;i10 3 0.3 8000 127 1
;i10 4 0.3 5000 127 1
;i101 0.0 5.124717 293.664764 0.000000 0.250000 2000.010010 0.250000 2000.010010 0.250000 2000.010010 4.000000
;i102 0.0 5.124717 293.664764 0.000000 4.000000
;i21	    0.0000	    1.0007	 25	127
;i20	    0	    0.3849	 20	127
;i 40	    0	    0.2448	 41	127
;i 30	    0	    0.9931	 30	127
;i 52	    0	    0.1942	 56	120
;i50 0 0.2 50 127
;i10 0 0.5 13 127
;i51 0 0.3 57 127
;i52 0 0.3 56 127

/*i101	0	4 	0	40 	6.00 
2015	2011	2015	2011	2015	2011	2015	2011 2015	2011	2015	2011	2015	2011	2015	2011	; table p7-22
1	0	1	0	1	0	1 0	1	0	1	0	1	0	1 0; pan	p23-p38

i101	0	4 	0	40 	6.07 
2015	2011	2015	2011	2015	2011	2015	2011 2015	2011	2015	2011	2015	2011	2015	2011	; table p7-22
0	1	0	1	0	1 0	1	0	1	0	1	0	1 0	1; pan	p23-p38
*/
/*i10 0 0.01 0 127 0.5
i10 2 0.03 0 127 0.5
i10 4 0.07 0 127 0.5
i10 6 0.1 0 127 0.5*/
/*i103 0 -1 32768 6.02
i104 2 -1 32768 6.09
i103 4  0 32768 6.09*/
;i104  0 6    7.00  

;i105 0  16.9   ;vox(horn1)

;i106   0    19
;i	107		0		5	
;i108    1   8 	
e	; end of score

</CsScore>
</CsoundSynthesizer><bsbPanel>
 <label>Widgets</label>
 <objectName/>
 <x>1520</x>
 <y>561</y>
 <width>400</width>
 <height>340</height>
 <visible>true</visible>
 <uuid/>
 <bgcolor mode="nobackground">
  <r>231</r>
  <g>46</g>
  <b>255</b>
 </bgcolor>
 <bsbObject version="2" type="BSBVSlider">
  <objectName>slider1</objectName>
  <x>5</x>
  <y>5</y>
  <width>20</width>
  <height>100</height>
  <uuid>{2ca409ef-1693-420f-8d99-efc0f191f90d}</uuid>
  <visible>true</visible>
  <midichan>0</midichan>
  <midicc>-3</midicc>
  <minimum>0.00000000</minimum>
  <maximum>1.00000000</maximum>
  <value>0.00000000</value>
  <mode>lin</mode>
  <mouseControl act="jump">continuous</mouseControl>
  <resolution>-1.00000000</resolution>
  <randomizable group="0">false</randomizable>
 </bsbObject>
</bsbPanel>
<bsbPresets>
</bsbPresets>
<MacGUI>
ioView nobackground {59367, 11822, 65535}
ioSlider {5, 5} {20, 100} 0.000000 1.000000 0.000000 slider1
</MacGUI>
