#!/bin/bash
while read line
do
	if [ -f $line ]
	then
		echo $line
	else
		dir=$(find -name $line)
		echo "try to find ${dir}"
		if [ -f $dir ]
		then
			echo $dir
		else
			echo "file do not exsit"
		fi
	fi
done < 1.txt
