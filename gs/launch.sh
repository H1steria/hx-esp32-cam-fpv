cd /home/pi/esp32-cam-fpv
cd gs
sudo airmon-ng check kill
sudo ip link set wlan1 down
sudo iw dev wlan1 set type monitor
sudo ip link set wlan1 up
sudo ./gs -fullscreen 1 -sm 1 -rx wlan1 -tx wlan1
