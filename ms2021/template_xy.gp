set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "x-Position (mm)"
set ylabel "y-Position (mm)"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 4:5 w lp ps 0