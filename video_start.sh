cd mjpg-streamer-experimental
export LD_LIBRARY_PATH=.
./mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -rot 180 -q 6 -x 640 -y 480 -fps 15"
