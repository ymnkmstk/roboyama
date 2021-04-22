#!/usr/bin/env bash
trap 'kill $(jobs -p)' EXIT

if [ -z "$ETROBO_ENV" ]; then
    echo "etrobo environment is not available."
    exit 1
fi

if [ "$1" = "right" ]; then
    LR="right"
else
    LR=""
fi

MAXTIME=65
DSTDIR=${ETROBO_HRP3_WORKSPACE}/ms2021/work
MAKELOG="makelog"
BTLOG="btlog"
COND="cond"
EXT="txt"
SEQ=1
SPEED=55
P=0.75D
I=0.39D
D=0.08D

cd $ETROBO_ROOT
if [ ! -d $DSTDIR ]; then
    mkdir -p $DSTDIR
fi

BASE=${MAKELOG}_${SEQ}

#for SPEED in 54 55 56; do
#for P in 0.24D 0.25D 0.26D 0.27D 0.28D 0.29D 0.30D; do # Ku = 0.28, Pu = 1.1 (1100ms)
# Kp = 0.6*Ku = 0.168, Ti = 0.5*Pu = 0.55, Td = 0.125*Pu = 0.1375
# Kp = 0.168, Ki = Kp/Ti = 0.3055, Kd = Kp*Td = 0.0231
#for D in 0.08D 0.09D; do
#for I in 0.35D 0.36D 0.37D 0.38D 0.39D; do
#for P in 0.7D 0.75D 0.8D; do
    for N in `seq 3`; do
        while ls $DSTDIR | grep -w $BASE >/dev/null; do
        SEQ=`expr $SEQ + 1`
        BASE=${MAKELOG}_${SEQ}
        done

        echo P=${P} I=${I} D=${D} Speed=${SPEED} > ${DSTDIR}/${COND}_${SEQ}.${EXT}
        export USER_COPTS="-DP_CONST=${P} -DI_CONST=${I} -DD_CONST=${D} -DSPEED_NORM=${SPEED}"
        btcat $LR > ${DSTDIR}/${BTLOG}_${SEQ}.${EXT} &
        timeout $MAXTIME make $LR app=2021base sim up 2>&1 | tee ${DSTDIR}/${MAKELOG}_${SEQ}.${EXT}
    done
#done

exit 0

timeout() {

    time=$1

    # start the command in a subshell to avoid problem with pipes
    # (spawn accepts one command)
    command="/bin/sh -c \"${@:2}\""

    expect -c "set timeout $time; spawn -noecho $command; expect timeout { exit 124 } eof; catch wait result; exit [lindex \$result 3]"
}