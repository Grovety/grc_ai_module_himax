#!/bin/bash

MIN_ITER=3
MAX_ITER=7

MIN_WAIT=10
MAX_WAIT=30

MAX_PASS=1000

for ((i=1; i<=$MAX_PASS; i++))
do
    ITER=$(($RANDOM % ($MAX_ITER - $MIN_ITER + 1) + $MIN_ITER))
    WAIT=$(($RANDOM % ($MAX_WAIT - $MIN_WAIT + 1) + $MIN_WAIT))
    python3 image_reader_bt.py -i$ITER
    for ((j=$WAIT; j>0; j--))
    do
        echo -n -e "$j     \r"
        sleep 1
    done
    echo Pass $i finished,$ITER iterations completed,waited for $WAIT second
done
