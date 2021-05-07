set terminal png
set nokey
set output "@PNGFILE"
set title "@TITLE"
set xlabel "f (Hz)"
set ylabel "|F(w)|"
set logscale x
set logscale y
plot [:][:] "@DATAFILE" using ($1/10):4 with impulses linewidth 1