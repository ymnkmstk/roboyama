set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Distance"
set ylabel "Sensor Value and Curvature"
plot "@DATAFILE" using 6:1 w lp ps 0, "@DATAFILE" using 6:2 w lp ps 0