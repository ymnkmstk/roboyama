set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Sensor Value and Curvature"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 1:2 w lp ps 0, "@DATAFILE" using 1:3 w lp ps 0