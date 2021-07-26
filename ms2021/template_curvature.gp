set terminal png
set output "@PNGFILE"
set title "@TITLE"
set xlabel "Time (ms)"
set ylabel "Sensor R Value, Curvature, MotorPower L and R"
set grid xtics mxtics ytics mytics
plot "@DATAFILE" using 1:2 w lp ps 0 lc 'red' ti "", "@DATAFILE" using 1:5 w lp ps 0 ti "", "@DATAFILE" using 1:10 w lp ps 0 ti "", "@DATAFILE" using 1:11 w lp ps 0 ti "" 