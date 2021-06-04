#!/usr/bin/env bash
if [ -z "$ETROBO_ENV" ]; then
    echo "etrobo environment is not available."
    exit 1
fi
if [ `uname` == "Darwin" ]; then
  GNUPLOT=/usr/local/bin/gnuplot
else
  GNUPLOT=/usr/bin/gnuplot
fi

DSTDIR=${ETROBO_HRP3_WORKSPACE}/ms2021/work
TMPL=${ETROBO_HRP3_WORKSPACE}/ms2021/template
SRC=${ETROBO_HRP3_WORKSPACE}/ms2021/fourier.c
OBJ=${ETROBO_HRP3_WORKSPACE}/ms2021/work/fourier.o
BTLOG="btlog"
COND="cond"
CMD="cmd"
DATA="fdata"
GRAPH="graph"
EXT="txt"
SEQ=1

BASE=${BTLOG}_${SEQ}
while ls $DSTDIR | grep -w $BASE >/dev/null; do
  LOGFILE=${DSTDIR}/${BASE}.${EXT}
  echo processing ${LOGFILE}...
  CONDFILE=${DSTDIR}/${COND}_${SEQ}.${EXT}
  DATAFILE=${DSTDIR}/${DATA}_${SEQ}.${EXT}
  gcc ${SRC} -o ${OBJ} -lfftw3
  # Use data for ten seconds as if it were for one second - do NOT forget to divide output frequency by ten!!! 
  cat $LOGFILE | grep "TraceLine::update()" | awk '{print $7}' | head -n 1000 | sed s:,::g | ${OBJ} > $DATAFILE
  gcc ${SRC} -o ${OBJ} -lfftw3
  for TYPE in fourier; do
    PNGFILE=${DSTDIR}/${GRAPH}_${SEQ}_${TYPE}.png
    CMDFILE=${DSTDIR}/${CMD}_${SEQ}_${TYPE}.gp
    cat ${TMPL}_${TYPE}.gp | sed "s:@TITLE:`cat ${CONDFILE}`:g" | sed "s:@PNGFILE:${PNGFILE}:g" | sed "s:@DATAFILE:${DATAFILE}:g" > $CMDFILE
    $GNUPLOT $CMDFILE
    rm $CMDFILE
  done
  SEQ=`expr $SEQ + 1`
  BASE=${BTLOG}_${SEQ}
done