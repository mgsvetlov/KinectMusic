<CsoundSynthesizer>

<CsOptions>
-odac0      -d     ;;;RT audio I/O
</CsOptions>

<CsInstruments>

sr = 44100
ksmps = 5
nchnls = 2
0dbfs = 2.0

        instr 1
kmidi chnget "midiPitch0"
kamp chnget "amp0"
kvibr chnget "vibrRate0"
kmod chnget "fmodSide0"

kcar = 1
kcps  = cpsmidinn(kmidi)
k1      oscil   kcps * 0.01, kvibr, 1
a1      foscil kamp, kcps + k1, 1, 2, kmod, 1
a2      oscil 0, 440, 1
outs     a1, a2
endin

instr 2
kmidi chnget "midiPitch1"
kamp chnget "amp1"
kvibr chnget "vibrRate1"
kmod chnget "fmodSide1"

kcar = 1
kcps  = cpsmidinn(kmidi)
k1      oscil   kcps * 0.01, kvibr, 1
a1      foscil kamp, kcps + k1, 1, 2, kmod, 1
a2      oscil 0, 440, 1
outs     a2, a1
endin

instr 100
endin
</CsInstruments>

<CsScore>
f1  0   4096    10 1  ; use GEN10 to compute a sine wave

;ins    strt    dur
i100      0     36000
e                     ; indicates the end of the score
</CsScore>

</CsoundSynthesizer>

