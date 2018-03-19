START=18
END=1000
for i in $(seq $START $END);
do
	mkdir output_log$i;
	cd output_log$i;
	mkdir frames;
	cd ..;
done
