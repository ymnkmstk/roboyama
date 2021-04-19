set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Distance"
set ylabel "Degree"
plot "@DATAFILE" using 6:5 w lp ps 0