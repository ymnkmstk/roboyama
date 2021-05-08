set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Sensor Value"
set grid xtics mxtics ytics mytics
plot [0:20000][0:100] "@DATAFILE" using 1:2 w lp ps 0