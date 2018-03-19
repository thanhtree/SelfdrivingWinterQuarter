NUM=$1 # which output log to save to
STRING="  int output_log_number = "$NUM"; // which output_log to save to"
# if output log is empty; ask user if he/she is sure to overwrite it
if [ "$(ls -A output_logs/output_log$NUM/frames)" ]; then
     read -n1 -p "output_log is not empty, do you wish to overwrite it? [y/n] " doit
     case $doit in
          y|Y) echo " yes ";
          cd output_logs/output_log$NUM/frames;
          sudo rm *; # remove everything
          cd ..; cd ..; cd ..;
          line72="`sed -n 72p controller.cpp`";
          if [ "$line72" != "$STRING" ]
          then
	      echo "replacing line";    
              sed -i "72s/.*/  int output_log_number = "$NUM"; \/\/ which output_log to save to/" controller.cpp # change output_log number
          fi
          echo "compiling";
          make
          #g++ $(pkg-config --libs --cflags opencv) -Wall -pthread -o controller controller.cpp joystick.cpp car_control.cpp -lpigpio -lrt -std=gnu++11 
	  echo "done compiling";
          echo "executing main program";
          echo " ";
          sudo ./controller;; 
          n|N) echo " no "  ;;
     esac
else
     echo "output_log is empty";
     line72="`sed -n 72p controller.cpp`"
     #echo $line72;
     #echo $STRING;
     if [ "$line72" != "$STRING" ] 
     then
         echo "replacing line";    
         sed -i "72s/.*/  int output_log_number = "$NUM"; \/\/ which output_log to save to/" controller.cpp # change output_log number
     fi
     echo "compiling";
     make
     #g++ $(pkg-config --libs --cflags opencv) -Wall -pthread -o controller controller.cpp joystick.cpp car_control.cpp -lpigpio -lrt -std=gnu++11
     echo "done compiling";
     echo "executing main program";
     echo " ";
     sudo ./controller

fi
