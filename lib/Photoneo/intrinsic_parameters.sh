#!/bin/bash
echo "proccessing press any key and ENTER to continue"
pushd `dirname "$0"` > /dev/null
bash set_params.sh >log.txt
popd > /dev/null

pushd `dirname "$0"` > /dev/null
echo "The intrinsic parameters were saved to intrinsic_parameters.txt file!"
(counter=0;
strindex() {
  x="${1%%$2*}"
  [[ "$x" = "$1" ]] ||  c=${#x}
}
while read LINE;
do #echo "asd $LINE";
counter=$(($counter+1));
#echo $counter$LINE
strindex "$LINE" " is "
num=$(($c+3))
if [ $counter -eq 2 ]
	then
	echo "The camera matrix is the following:"$'\n'
	fi
if [ $counter -eq 12 ]
	then
	echo $'\n'
	echo "The distortion coefficients are the following:"$'\n' ;
	fi
if [ $counter -gt 2 ] && [ $counter -lt 26 ]
	then
	echo ${LINE: $num} ;
	fi
if 	[ $counter -eq 3 ]
	then
	f_x=${LINE: $num}
	fi
if 	[ $counter -eq 7 ]
	then
	f_y=${LINE: $num}
	fi

done <  log.txt


)> intrinsic_parameters.txt
popd > /dev/null
