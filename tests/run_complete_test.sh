#!/bin/bash
STEPS=100
for TEST in simple_circle_motion
do
    for NUM in 1 #2 3 4 5 6 7 8 9 10
    do
        echo "$TEST $NUM running Camera"
        python3 $TEST.py --steps $STEPS -HF -AMCL -SDL_HF -SDL_AMCL -can_measure_r -can_measure_a --sub_dir camera
        echo "$TEST $NUM running LBL"
        python3 $TEST.py --steps $STEPS -HF -AMCL -SDL_HF -SDL_AMCL -can_measure_r --sub_dir lbl
        echo "$TEST $NUM running Bearing"
        python3 $TEST.py --steps $STEPS -HF -AMCL -SDL_HF -SDL_AMCL -can_measure_a --sub_dir bearing
    done 
done
