<CsoundSynthesizer>
<CsOptions>
; Select audio/midi flags here according to platform
-odac0      -d     ;;;RT audio I/O
;-iadc    ;;;uncomment -iadc if realtime audio input is needed too
; For Non-realtime ouput leave only the line below:
; -o loscil.wav -W ;;; for file output any platform
</CsOptions>
<CsInstruments>

sr = 44100
ksmps = 10
nchnls = 2
0dbfs = 2.0

instr 1

kamp chnget "amp1"

ichnls = ftchnls(p5)
print ichnls

if (ichnls == 1) then
   asigL loscil .8, 1, p4, 1
   asigR = 	asigL
elseif (ichnls == 2) then
   asigL, asigR loscil kamp, 1, p4, 1
;safety precaution if not mono or stereo
else
   asigL = 0
   asigR = 0
endif
   outs asigL, asigR
endin

instr 2

kamp chnget "amp2"

ichnls = ftchnls(p5)
print ichnls

if (ichnls == 1) then
asigL loscil .8, 1, p4, 1
asigR = 	asigL
elseif (ichnls == 2) then
asigL, asigR loscil kamp, 1, p4, 1
;safety precaution if not mono or stereo
else
asigL = 0
asigR = 0
endif
outs asigL, asigR
endin

instr 100
endin

</CsInstruments>
<CsScore>
f 1 0 0 1 "sounds/whistle/001.aiff" 0 0 0		;2.132
f 2 0 0 1 "sounds/whistle/002.aiff" 0 0 0		;1.995
f 3 0 0 1 "sounds/door/001.aiff" 0 0 0 		;3.957
f 4 0 0 1 "sounds/door/002.aiff" 0 0 0 		;7.486

i100      0     36000

;i 1 0 2.132 1 2 ;stereo file
;i 1 + 1.995 2 2 ;stereo file
;i 1 + 3.957 3 2 ;stereo file
;i 1 + 7.486 4 2 ;stereo file
e
</CsScore>
</CsoundSynthesizer>
<bsbPanel>
 <label>Widgets</label>
 <objectName/>
 <x>100</x>
 <y>100</y>
 <width>320</width>
 <height>240</height>
 <visible>true</visible>
 <uuid/>
 <bgcolor mode="nobackground">
  <r>255</r>
  <g>255</g>
  <b>255</b>
 </bgcolor>
</bsbPanel>
<bsbPresets>
</bsbPresets>
