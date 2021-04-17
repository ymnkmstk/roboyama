#!/bin/sh
trap 'kill $(jobs -p)' EXIT

if [ -z "$ETROBO_ENV" ]; then
    echo "etrobo environment is not available."
    exit 1
fi

MAXTIME=60
DSTDIR=${ETROBO_HRP3_WORKSPACE}/ms2021/work
MAKELOG="makelog"
BTLOG="btlog"
EXT="txt"
SEQ=1

cd $ETROBO_ROOT
if [ ! -d $DSTDIR ]; then
    mkdir -p $DSTDIR
fi

BASE=${MAKELOG}_${SEQ}
while ls $DSTDIR | grep -w $BASE >/dev/null; do
  SEQ=`expr $SEQ + 1`
  BASE=${MAKELOG}_${SEQ}
done

for P in 0.85 0.65; do
    for SPEED in 50 25; do
        export USER_COPTS="-DP_CONST=${P} -DSPEED_NORM=${SPEED}"
        btcat > ${DSTDIR}/${BTLOG}_${SEQ}.${EXT} &
        timeout $MAXTIME make app=2021base sim up 2>&1 | tee ${DSTDIR}/${MAKELOG}_${SEQ}.${EXT}
    done
done

exit 0

timeout() {

    time=$1

    # start the command in a subshell to avoid problem with pipes
    # (spawn accepts one command)
    command="/bin/sh -c \"${@:2}\""

    expect -c "set timeout $time; spawn -noecho $command; expect timeout { exit 124 } eof; catch wait result; exit [lindex \$result 3]"
}