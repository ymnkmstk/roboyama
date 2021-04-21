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
BTLOG="btlog"
COND="cond"
CMD="cmd"
DATA="data"
GRAPH="graph"
EXT="txt"
SEQ=1

BASE=${BTLOG}_${SEQ}
while ls $DSTDIR | grep -w $BASE >/dev/null; do
  LOGFILE=${DSTDIR}/${BASE}.${EXT}
  echo processing ${LOGFILE}...
  CONDFILE=${DSTDIR}/${COND}_${SEQ}.${EXT}
  DATAFILE=${DSTDIR}/${DATA}_${SEQ}.${EXT}
  cat $LOGFILE | grep "TraceLine::update()" | awk '{print $1/1000, $7, $10, $13, $16, $19, $22}' | sed s:,::g > $DATAFILE
  for TYPE in xy degree curvature distance; do
    PNGFILE=${DSTDIR}/${GRAPH}_${SEQ}_${TYPE}.png
    CMDFILE=${DSTDIR}/${CMD}_${SEQ}_${TYPE}.gp
    cat ${TMPL}_${TYPE}.gp | sed "s:@TITLE:`cat ${CONDFILE}`:g" | sed "s:@PNGFILE:${PNGFILE}:g" | sed "s:@DATAFILE:${DATAFILE}:g" > $CMDFILE
    $GNUPLOT $CMDFILE
    rm $CMDFILE
  done
  rm $DATAFILE
  SEQ=`expr $SEQ + 1`
  BASE=${BTLOG}_${SEQ}
done
