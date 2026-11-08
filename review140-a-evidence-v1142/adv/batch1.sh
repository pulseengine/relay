#!/bin/bash
A=/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad/adv
$A/run2.sh rv_adv rv_adv a1P 1
RV_RATES=110,120,128,133,140 RV_VIBS=0,0.3 RV_PATS=fixed,alt,bp3,p5,late10,late5,drift,jit RV_SCNS=long,rotorout $A/run2.sh rv_adv rv_adv nyq
RV_RATES=250,400 RV_VIBS=0.35,0.4,0.5 RV_PATS=late5,mb,late40 RV_SCNS=long RV_SEEDS=48 $A/run2.sh rv_adv rv_adv l5
