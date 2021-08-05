#!/usr/bin/env bash

##ltloopr.sh
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

usage_exit() {
        echo "  maxtime make開始から走行打ち切りまでの実時間" 1>&2
        echo "" 1>&2
        echo "  ログは${DSTDIR}に格納されています" 1>&2
        exit 1
}

SRCDIR=${ETROBO_HRP3_WORKSPACE}/ms2021
DSTDIR=${ETROBO_ROOT}/workspace/ms2021/work

MAKELOG="makelog"
LOOPLOG="lplog"
LOGEXT="txt"
CSVEXT="csv"
DT=`date "+%y%m%d%H%M%S"`

#実行ディレクトリはETROBOのroot
cd $ETROBO_ROOT

#処理打ち切り時間がパラメータにあったらセット
if [ $# -eq 0 ];then
    MAXTIME=50
else
    MAXTIME=$1
fi

# 最初に1回だけmake
make right app=ms2021 sim 2>&1 | tee ${DSTDIR}/${MAKELOG}_${DT}.${LOGEXT}

# 処理ループ
for ((ll = 0; ll <= 3; ll++)) {
    for ((rot = 0; rot <= 3; rot++)) {
        for ((rsp = 0; rsp <= 3; rsp++)) {
# シミュレータ起動と初期位置設定
            sim right ctl pos -3 0 -16.35 -90
            sleep 2
# 光源値設定
            curl -X POST -H "Content-Type: application/json" -d "{\"EnvLightIntensityLevel\":$ll,\"EnvLightRotation\":$rot,\"LSpotLight\":"0",\"RSpotLight\":$rsp}" http://localhost:54000
#            curl -X POST -H "Content-Type: application/json" -d "{\"EnvLightIntensityLevel\":"0",\"EnvLightRotation\":"0",\"LSpotLight\":"0",\"RSpotLight\":"0"}" http://localhost:54000
            sleep 2
# アプリを実行しプロセスIDを記録
            asp right ms2021 &
            PID=`asp check right`
# シミュレータ PREPAREモード
            sim right ctl prepare
            sleep 2
# シミュレータ GOモード
            sim right ctl go &
# 処理打ち切り時間を越えたらアプリプロセスの動作確認、プロセスが居たら殺す
            sleep $MAXTIME
            CNT=0
            CNT=`ps -ef | grep $PID | wc -l`
            if [ $CNT -ne 0 ]; then
                kill $PID
            fi
            wait $PID
# 終了処理
            echo "stop"
            sleep 1
            sim right ctl end 2>&1 | tee ${DSTDIR}/lp_${ll}${rot}0${rsp}.${CSVEXT}
            asp stop
         }
     }
}
