#!/bin/bash
S=/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad
cd $S
G() { ./rv_run_tree.sh $S/lockA-proto $S/td-proto "$@"; }
G rv_probe9 rv_grid9 g16_9
G rv_probe8 rv_grid8 g16_8
G rv_altgrid rv_altgrid g16_alt
G rv_adv rv_adv g16_a1
RV_RATES=110,120,128,133,140 RV_VIBS=0,0.3 RV_PATS=fixed,alt,bp3,p5,late10,late5,drift,jit RV_SCNS=long,rotorout G rv_adv rv_adv g16_nyq
RV_RATES=250,400 RV_VIBS=0.35,0.4,0.5 RV_PATS=late5,mb,late40 RV_SCNS=long RV_SEEDS=48 G rv_adv rv_adv g16_l5
echo DONE
