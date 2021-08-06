BEGIN {
    FS = ","
}
{
#右コース用です
#ファイル名から光源設定値を抽出
    FNAME = substr(FILENAME,index(FILENAME,".csv")-4,4)
    printf "%s%c",FNAME,","
#配列にデータをセット
    FLG = 0
    for(i = 1 ; i <= NF ; i++) {
#        print "$i="$i
        split( $i , word , ":")
#       print "word(1)"word[1]"word(2)"word[2]
        gsub("\"","",word[1])
#        print "word(1)"word[1]"word(2)"word[2]
        if (word[1] == "rightMeasurement") {
            FLG = 1
        }
        if (FLG == 1) {
            Set[word[1]] = word[2]
        }
    }
#データを出力
##計測値
    if (Set["MEASUREMENT_TIME"] == 0) {
        printf "%s","0,"
    }
    else {
        printf "%s%c%s%c",substr(Set["MEASUREMENT_TIME"],1,2),".",substr(Set["MEASUREMENT_TIME"],3,3),","
    }
##走行値
    if (Set["RUN_TIME"] == 0) {
        printf "%s","0,"
    }
    else {
        printf "%s%c%s%c",substr(Set["RUN_TIME"],1,2),".",substr(Set["RUN_TIME"],3,3),","
    }
##ゲート1通過
    printf "%s%c",Set["GATE1"],","
##ゲート2通過
    printf "%s%c",Set["GATE2"],","
##ゴール通過
    printf "%s%c",Set["GOAL"],","
##スラローム通過
    printf "%s%c",Set["SLALOM"],","
##ペットボトル本数
    printf "%s%c",Set["PETBOTTLE"],","
##ガレージ停止
    printf "%s%c",Set["GARAGE_STOP"],","
##ガレージ停止時間
    if (Set["GARAGE_TIME"] == 0) {
        printf "%s","0,"
    }
    else {
        printf "%s%c%s%c",substr(Set["GARAGE_TIME"],1,1),".",substr(Set["GARAGE_TIME"],2,3),","
    } 
##ブロック搬入 
    print Set["BLOCK_IN_GARAGE"] 
}
