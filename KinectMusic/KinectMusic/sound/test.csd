<CsoundSynthesizer>

<CsOptions>
-odac0      -d     ;;;RT audio I/O
</CsOptions>

<CsInstruments>

sr = 44100
ksmps = 128
nchnls = 2
0dbfs = 2.0

        instr 1
kfreq chnget "pitch0"
kamp chnget "vol0"
a1      oscil   kamp, kfreq, 1
        out     a1
        endin

        instr 2
kfreq chnget "pitch1"
kamp chnget "vol1"
a1      oscil   kamp, kfreq, 1
out     a1
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

