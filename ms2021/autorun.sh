#!/usr/bin/env bash
trap 'killbackground > /dev/null 2>&1' EXIT

timeout() {
    time=$1

    # start the command in a subshell to avoid problem with pipes
    # (spawn accepts one command)
    command="/bin/sh -c \"${@:2}\""

    expect -c "set timeout $time; spawn -noecho $command; expect timeout { exit 124 } eof; catch wait result; exit [lindex \$result 3]"
}

killbackground(){
    local jobs=`jobs -p`
    for job in $jobs; do
	@@ -31,47 +21,23 @@ if [ -z "$ETROBO_ENV" ]; then
    exit 1
fi

if [ "$1" = "right" ]; then
    LR="right"
else
    LR=""
fi

MAXTIME=30
DSTDIR=${ETROBO_HRP3_WORKSPACE}/ms2021/work
MAKELOG="makelog"
BTLOG="btlog"
COND="cond"
EXT="txt"
SEQ=1
SPEED=55
P=0.75
I=0.39
D=0.08

cd $ETROBO_ROOT
if [ ! -d $DSTDIR ]; then
    mkdir -p $DSTDIR
fi

BASE=${MAKELOG}_${SEQ}

for SPEED in 54 55 56; do
#for P in 0.24D 0.25D 0.26D 0.27D 0.28D 0.29D 0.30D; do # Ku = 0.28, Pu = 1.1 (1100ms)
# Kp = 0.6*Ku = 0.168, Ti = 0.5*Pu = 0.55, Td = 0.125*Pu = 0.1375
# Kp = 0.168, Ki = Kp/Ti = 0.3055, Kd = Kp*Td = 0.0231
#for D in 0.08 0.09; do
#for I in 0.35 0.36 0.37 0.38 0.39; do
#for P in 0.7 0.75 0.8; do
    for N in `seq 2`; do
        while ls $DSTDIR | grep -w $BASE >/dev/null; do
        SEQ=`expr $SEQ + 1`
        BASE=${MAKELOG}_${SEQ}
        done

        echo P=${P} I=${I} D=${D} Speed=${SPEED} > ${DSTDIR}/${COND}_${SEQ}.${EXT}
        export USER_COPTS="-DP_CONST=${P}D -DI_CONST=${I}D -DD_CONST=${D}D -DSPEED_NORM=${SPEED}"
        btcat $LR > ${DSTDIR}/${BTLOG}_${SEQ}.${EXT} &
        timeout $MAXTIME make $LR app=ms2021 sim up 2>&1 | tee ${DSTDIR}/${MAKELOG}_${SEQ}.${EXT}
    done
done