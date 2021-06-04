set terminal png
set nokey
set output "@PNGFILE"
set title "@TITLE"
set xlabel "f (Hz)"
set ylabel "|F(w)|"
plot [:][:5000] "@DATAFILE" using ($1/10):4 with impulses linewidth 1