#!/bin/bash
S=/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad
cd $S
export RV_RATES=250,400,1000 RV_VIBS=0,0.3 RV_SEEDS=8
export RV_PATS=alt,flap16,flap17,flap24,flap32,flap64,flap32m3,flap16m16,alt23,alt5,altoff,altoffd,l3,l3b,l3c,saw,asym,asym2,asym3,altdrift2,altdrift10,altamp,altlate,jit15,jit30,jit50,altbad1,altbad5,mb2,mbl2
export RV_SCNS=long,rotorout,second,cycles,swfix,swalt,fixsw,step
./adv/run2.sh rv_adv2 rv_adv2 _F
./adv/run2.sh rv_adv2 rv_adv2 _FP 1
echo DONE
