Burn 5.3 image
sudo apt update


Sbitx_ctrl install
Open sbitx_soapy_pihpsdr_files folder, inside there is another folder called sbitx-core_mod
Open terminal from this folder
sudo apt install libi2c-dev 
gcc -O2 -pthread -o sbitx_ctrl  sbitx_ctrl.c sbitx_core.c sbitx_gpio.c sbitx_i2c.c sbitx_si5351.c   -lwiringPi -li2c
cd /etc
sudo mkdir sbitx
Copy the file "alsamix_bands.ini" from the sbitx-core_mod folder to the /etc/sbitx folder
run it  ./sbitx_ctrl
If you want to test sbitx_ctrl install
sudo apt install netcat-traditional
run nc 127.0.0.1 9999
f  --view freq
F 14200000 to set freq to 14.200mhz
T 1 set ptt
T 0 rx
t ptt status


Soapy driver install
first install
sudo apt install -y soapysdr-tools libsoapysdr-dev
sudo apt install -y libliquid-dev





go to  soapy_sbitx_driver folder
mkdir -p build
cd build
cmake ..
make -j2
sudo make install
sudo ldconfig

Test driver install, on available factories you should see sbitx listed
SoapySDRUtil --info
SoapySDRUtil --probe="driver=sbitx"

pihpsdr dl1ycf install
cd 
git clone https://github.com/dl1ycf/pihpsdr
Go to pihpsdr folder
Copy gpio.props file
Copy sbitx.props file
Edit makefile with this options and save:
GPIO=ON
MIDI=ON
SATURN=OFF
USBOZY=OFF
SOAPYSDR=ON
STEMLAB=OFF
AUDIO=ALSA
NR34LIB=OFF
TTS=OFF

go to pihpsdr/src folder
copy soapy_discovery.c from file and replace the one on pihpsdr folder.
save and close
on the same folder open sliders.c
go to line 1008    adc[0].min_gain, adc[0].max_gain, 1.0);
change to 	   adc[0].min_gain, adc[0].max_gain, 0.01);

go to line 1012    gtk_range_set_increments (GTK_RANGE(rf_gain_scale), 1.0, 1.0);
change to 	   gtk_range_set_increments (GTK_RANGE(rf_gain_scale), 0.01, 0.01);
save and close

go back to pihpsdr folder
sudo apt-get --yes install libgpiod-dev
make -j4


The soapy_discovery.c file replacement is necessary to be able to have the tx ability. Recent changes leave the sbitx driver unable to tx.
gpio props file enable controller 1 setup to operate encoders, lower encoder vfo, upper encoder volume. Upper encoder function can be changed 
on the encoders menu.  
sbitx.props sets the rfgain limits to 0 and 1. step is .01. Also sets the speaker audio to loopback 1,0 and mic to 2,0.
On the sbitx-core_mod folder, there is a python program called sbitx_ini_gui.py This is to help adjusting the tx rx audio levels. This is more like
a coarse adjustment. On pihpsdr txdrive adjustment and rfgain adjustment give the relative adjustent for normal use.
If you create a desktop icon for pihpsdr, you need to copy the gpio.props and sbitx.props files to /home/pi/.config/pihpsdr



If you dont run sbitx app first, then before running, you need to go to alsamixer and change capture from mic to line
Need to use a usb headset or usb audio device. Sbitx speaker and mic are not yet integrated.

