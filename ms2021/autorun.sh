#!/usr/bin/env bash
trap 'killbackground > /dev/null 2>&1' EXIT

killbackground(){
    local jobs=`jobs -p`
    for job in $jobs; do
        killpstree $job
    done
}

killpstree(){
    local children=`pgrep -P $1`
    for child in $children; do
        killpstree $child
    done
    kill $1
}

if [ -z "$ETROBO_ENV" ]; then
    echo "etrobo environment is not available."
    exit 1
fi

LR=
#LR="-r"
COUNT=1
MAXTIME=30
SPEED=55
P=0.75
I=0.39
D=0.08

I=0.0
D=0.0
for P in 0.1 0.2 0.3 0.4 0.5 0.6 0.7; do # Ku = 0.28, Pu = 1.1 (1100ms)
# Kp = 0.6*Ku = 0.168, Ti = 0.5*Pu = 0.55, Td = 0.125*Pu = 0.1375
# Kp = 0.168, Ki = Kp/Ti = 0.3055, Kd = Kp*Td = 0.0231
#for D in 0.08 0.09; do
#for I in 0.35 0.36 0.37 0.38 0.39; do
#for P in 0.7 0.75 0.8; do
    $ETROBO_HRP3_WORKSPACE/ms2021/run.sh $LR -c $COUNT -t $MAXTIME -s $SPEED -p $P -i $I -d $D
done
$ETROBO_HRP3_WORKSPACE/ms2021/graph.sh