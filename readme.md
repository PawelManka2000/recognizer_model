Project is based on Articubot from Articulated Robotics 


udevadm info --name=/dev/ttyACM0 --attribute-walk 

sudo nano /etc/udev/rules.d/99-usb-serial.rules 

SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", ATTRS{serial}=="0055002F3432511330343838", SYMLINK+="stmG4" 

 

Sudo udevadm control –reload-rules 

Sudo udevadm trigger 

 
