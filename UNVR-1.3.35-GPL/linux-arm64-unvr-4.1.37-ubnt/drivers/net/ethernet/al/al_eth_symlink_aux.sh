#!/bin/bash
#usage $1 - Path, $2 - Files, $3 - Symlink or remove

for FILE_NAME in $2
do
	if [ $3 == "SYM" ]; then
		ln -sfn $1/$FILE_NAME ./$FILE_NAME
	elif [ -f $FILE_NAME ]; then
		rm $FILE_NAME
	fi
done
