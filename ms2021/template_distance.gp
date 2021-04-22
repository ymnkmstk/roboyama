set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Distance (mm)"
plot "@DATAFILE" using 1:7 w lp ps 0