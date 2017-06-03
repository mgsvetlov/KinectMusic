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

        out     a1
        endin

</CsInstruments>

<CsScore>
f1  0   4096    10 1  ; use GEN10 to compute a sine wave

;ins    strt    dur
i1      0       3600

e                     ; indicates the end of the score
</CsScore>

</CsoundSynthesizer>

