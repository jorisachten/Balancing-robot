echo "remove old version"
sudo rm BALANCE
echo "start build"
gcc -o BALANCE BALANCE.cpp COMMUNICATION_BUS.cpp wiringPiSPI.cpp -lwiringPi I2Cdev.cpp MPU6050.cpp JOYSTICK_CLIENT.cpp mongoose.cpp -lpthread
echo "start program"
sudo bash start.sh
