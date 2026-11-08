#!/bin/bash
S=/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad
cd $S
export RV_VIBS=0.15,0.3,0.5 RV_SEEDS=8 RV_SCNS=long
RV_RATES=250 RV_PATS=altc100a60,altc100a125,altc100a200,fixc100,altc110a60,altc110a125,altc110a200,fixc110,altc120a60,altc120a125,altc120a200,fixc120,altc130a60,altc130a125,altc130a200,fixc130,altc140a60,altc140a125,altc140a200,fixc140,altc150a60,altc150a125,altc150a200,fixc150,altc160a60,altc160a125,altc160a200,fixc160 ./adv/run2.sh rv_adv2 rv_adv2 _C
RV_RATES=250 RV_PATS=altc100a60,altc100a125,altc100a200,fixc100,altc110a60,altc110a125,altc110a200,fixc110,altc120a60,altc120a125,altc120a200,fixc120,altc130a60,altc130a125,altc130a200,fixc130,altc140a60,altc140a125,altc140a200,fixc140,altc150a60,altc150a125,altc150a200,fixc150,altc160a60,altc160a125,altc160a200,fixc160 ./adv/run2.sh rv_adv2 rv_adv2 _CP 1
RV_RATES=400 RV_PATS=altc160a60,altc160a125,fixc160,altc180a60,altc180a125,fixc180,altc200a60,altc200a125,fixc200,altc220a60,altc220a125,fixc220,altc240a60,altc240a125,fixc240 ./adv/run2.sh rv_adv2 rv_adv2 _C4
RV_RATES=400 RV_PATS=altc160a60,altc160a125,fixc160,altc180a60,altc180a125,fixc180,altc200a60,altc200a125,fixc200,altc220a60,altc220a125,fixc220,altc240a60,altc240a125,fixc240 ./adv/run2.sh rv_adv2 rv_adv2 _C4P 1
echo DONE
