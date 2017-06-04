<CsoundSynthesizer>

<CsOptions>
-odac0      -d     ;;;RT audio I/O
</CsOptions>

<CsInstruments>

sr = 44100
ksmps = 10
nchnls = 2
0dbfs = 2.0

        instr 1
kmidi chnget "p10"
kamp chnget "p11"
kvibr chnget "p12"
kmod chnget "p13"

kcar = 1
kcps  = cpsmidinn(kmidi)
k1      oscil   kcps * 0.01, kvibr, 1
a1      foscil kamp, kcps + k1, 1, 2, kmod, 1
a2      oscil 0, 440, 1
outs     a1, a2
endin

instr 2
kmidi chnget "p20"
kamp chnget "p21"
kvibr chnget "p22"
kmod chnget "p23"

kcar = 1
kcps  = cpsmidinn(kmidi)
k1      oscil   kcps * 0.01, kvibr, 1
a1      foscil kamp, kcps + k1, 1, 2, kmod, 1
a2      oscil 0, 440, 1
outs     a2, a1
endin

</CsInstruments>

<CsScore>
f1  0   4096    10 1  ; use GEN10 to compute a sine wave

;ins    strt    dur
i1      0       3600
i2      0       3600
e                     ; indicates the end of the score
</CsScore>

</CsoundSynthesizer>

