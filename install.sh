#!/bin/bash
# ------------------------------------------------
# sBitx + SoapySDR + piHPSDR Full Install Script
# Usage: chmod +x install.sh && sudo ./install.sh
# ------------------------------------------------

set -e

INSTALL_USER=${SUDO_USER:-$USER}
INSTALL_HOME=$(eval echo "~$INSTALL_USER")

echo "=== Running as user: $INSTALL_USER ==="
echo "=== Home directory: $INSTALL_HOME ==="

echo "=== Updating system ==="
sudo apt update
sudo apt install -y \
    git build-essential cmake wiringpi libi2c-dev \
    soapysdr-tools libsoapysdr-dev libliquid-dev \
    libgpiod-dev netcat-traditional

###############################################################################
# Clone / Update sbitx-soapy
###############################################################################
echo
echo "=== Installing sbitx-soapy ==="
cd "$INSTALL_HOME"

if [ -d "sbitx-soapy" ]; then
    cd sbitx-soapy
    git pull
else
    git clone https://github.com/drexjj/sbitx-soapy.git
    cd sbitx-soapy
fi

###############################################################################
# SECTION 1 — Build sbitx_ctrl
###############################################################################
echo
echo "=== Building sbitx_ctrl ==="
cd sbitx-core_mod

gcc -O2 -pthread -o sbitx_ctrl \
    sbitx_ctrl.c sbitx_core.c sbitx_gpio.c \
    sbitx_i2c.c sbitx_si5351.c \
    -lwiringPi -li2c

echo "=== Installing sbitx ALSA band config ==="
sudo mkdir -p /etc/sbitx
sudo cp alsamix_bands.ini /etc/sbitx/

###############################################################################
# SECTION 2 — Build SoapySDR sBitx driver
###############################################################################
echo
echo "=== Building SoapySDR sBitx driver ==="
cd ../soapy_sbitx_driver
mkdir -p build
cd build

cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig

###############################################################################
# SECTION 3 — Build piHPSDR
###############################################################################
echo
echo "=== Installing piHPSDR ==="
cd "$INSTALL_HOME"

if [ -d "pihpsdr" ]; then
    cd pihpsdr
    git pull
else
    git clone https://github.com/dl1ycf/pihpsdr
    cd pihpsdr
fi

# Patch the Makefile to configure features the way we want
if git diff --quiet Makefile; then
    # generate the patch for the code
    cat > Makefile.diff <<EOF
--- a/Makefile
+++ b/Makefile
@@ -12,11 +12,11 @@
 #
 #######################################################################################
 
-GPIO=ON
+GPIO=OFF
 MIDI=ON
-SATURN=ON
+SATURN=OFF
 USBOZY=OFF
-SOAPYSDR=OFF
+SOAPYSDR=ON
 STEMLAB=OFF
 AUDIO=PULSE
 NR34LIB=OFF
EOF
    # apply the patch for the code
    patch -p1 < Makefile.diff
else
    echo "Makefile is already modified"
fi

echo "=== Copying props files ==="
cp "$INSTALL_HOME/sbitx-soapy/pihpsdr_files/gpio.props" .
cp "$INSTALL_HOME/sbitx-soapy/pihpsdr_files/sbitx.props" .

echo "=== Installing TX-capable soapy_discovery.c ==="
cp "$INSTALL_HOME/sbitx-soapy/pihpsdr_files/soapy_discovery.c" src/

echo "=== Adjusting RF gain slider resolution ==="
sed -i \
    's/adc\[0\]\.min_gain, adc\[0\]\.max_gain, 1\.0/adc[0].min_gain, adc[0].max_gain, 0.01/' \
    src/sliders.c

sed -i \
    's/gtk_range_set_increments (GTK_RANGE(rf_gain_scale), 1\.0, 1\.0)/gtk_range_set_increments (GTK_RANGE(rf_gain_scale), 0.01, 0.01)/' \
    src/sliders.c

echo "=== Building piHPSDR ==="
make -j$(nproc)

###############################################################################
# FINAL
###############################################################################
echo
echo "=== Installation complete ==="
echo "* sbitx_ctrl binary location: sbitx-core_mod/sbitx_ctrl"
echo "* ALSA config in: /etc/sbitx"
echo "* SoapySDR driver available to SoapySDRUtil"
echo "* piHPSDR built in pihpsdr_files/src"
echo
echo "Next steps:"
echo "1) Run sbitx_ctrl: cd ~/sbitx-soapy/sbitx-core_mod && ./sbitx_ctrl"
echo "2) Launch piHPSDR from build directory."
echo "3) Optionally copy gpio.props and sbitx.props to ~/.config/pihpsdr for desktop launchers."

echo "=== Testing SoapySDR drivers ==="
SoapySDRUtil --info | grep sbitx || true
SoapySDRUtil --probe=driver=sbitx || true

echo
echo "Done. Reboot recommended."
