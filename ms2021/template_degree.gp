set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Degree"
plot "@DATAFILE" using 1:6 w lp ps 0