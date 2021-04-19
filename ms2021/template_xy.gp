set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "x-Position"
set ylabel "y-Position"
plot "@DATAFILE" using 3:4 w lp ps 0