set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Distance (mm)"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 1:7 w lp ps 0