set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "x-Position (mm)"
set ylabel "y-Position (mm)"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 6:7 w lp ps 0 ti ""