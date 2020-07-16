#!/bin/sh
sed -nE '/Observer.*distance.*azimuth/{s/^.*x = ([0-9-]+), y = ([0-9-]+).*$/\1,\2/;p};/Captain default constructor/s/.*/cut/p' | (
    for num in `seq -w 000 999` ; do
	while read line ; do
	    if [ $line = 'cut' ] ; then
		file=file$num.csv
		echo "x,y" > $file
		break
	    fi
	    echo $line >> $file
	done
    done
)
