set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Sensor RGB Value"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 1:2 w lp ps 0 lc 'red' ti "", "@DATAFILE" using 1:3 w lp ps 0 lc 'green' ti "", "@DATAFILE" using 1:4 w lp ps 0 lc 'blue' ti "" 