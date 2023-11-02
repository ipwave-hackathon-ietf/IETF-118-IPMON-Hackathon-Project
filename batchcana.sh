#! /bin/sh

./configure

make MODE=release

cd CANA-Dyn/simulations/simu5gdrone

for i in 0 1 2 3 4
 do
	for j in 1 2 3 4 5 6
	do
		./run -u Cmdenv -f omnetppstatic.ini -r $i -c NumDrone_$j
	
		mv results/collprobres.csv results/collprobres"$i"_"$j".csv
		mv results/accidentres.csv results/accidentres"$i"_"$j".csv

	done
done
date

