#!/bin/bash

HOSTS_FILE="/etc/hosts"
HOST_IP="192.168.1.39"
HOST_NAME="vicon"

RPI_CONFIG_FILE="/boot/firmware/config.txt"

PWM_CONFIG="dtoverlay=pwm"
UART_CONFIG_1="enable_uart=1"
UART_CONFIG_2="dtoverlay=disable-bt,uart0,ctsrts"
CAMERA_CONFIG="dtoverlay=ov9281,cam0"
LED_CONFIG="dtparam=spi=on"

CONFIGS_TO_ADD=(
    "$PWM_CONFIG"
    "$UART_CONFIG_1"
    "$UART_CONFIG_2"
    "$CAMERA_CONFIG"
    "$LED_CONFIG"
)

CAM_LOC_REPO="https://github.com/flslab/fls-marker-localization.git"
CAM_LOC_REPO_DIR="$HOME/fls-marker-localization"

sudo -v

# Adding configs

if [ ! -f "$RPI_CONFIG_FILE" ]; then
    echo "Error: File $RPI_CONFIG_FILE not found."
    exit 1
fi

echo "Checking configuration in $RPI_CONFIG_FILE..."

for config in "${CONFIGS_TO_ADD[@]}"; do
    # -F: Fixed string (not regex)
    # -x: Exact line match
    # -q: Quiet (no output)
    if grep -Fxq "$config" "$RPI_CONFIG_FILE"; then
        echo "  [OK] Already exists: $config"
    else
        echo "  [UPDATE] Adding: $config"
        # Echo the config line and pipe it into sudo tee -a (append)
        # > /dev/null suppresses the output of tee to the screen
        echo "$config" | sudo tee -a "$RPI_CONFIG_FILE" > /dev/null
    fi
done

echo ""

# Setting the Vicon host name
echo "Checking $HOSTS_FILE..."

# Check using regex (-e):
# ^          = Start of line (prevents matching 1192.168...)
# [[:space:]]+ = One or more spaces or tabs
if grep -q -e "^$HOST_IP[[:space:]]\+$HOST_NAME" "$HOSTS_FILE"; then
    echo "  [OK] Entry already exists: $HOST_NAME"
else
    echo "  [UPDATE] Adding: $HOST_IP $HOST_NAME"
    # printf generates the line with a tab (\t) and a newline (\n)
    # We pipe it to 'sudo tee -a' to write to the protected file
    printf "$HOST_IP\t$HOST_NAME\n" | sudo tee -a "$HOSTS_FILE" > /dev/null
fi

mkdir logs -p

# Install motioncapture lib dependencies
sudo apt install libboost-system-dev libboost-thread-dev libeigen3-dev ninja-build
pip install -r requirements.txt
pip install -r requirements_servo.txt
pip install -r requirements_led.txt


# Define variables for modified libmotioncaputre
REPO_URL="https://github.com/Hamedamz/libmotioncapture.git"
REPO_DIR="libmotioncapture"
PACKAGE_NAME="motioncapture"

echo "Step 1: Uninstalling existing $PACKAGE_NAME library..."
pip uninstall -y $PACKAGE_NAME

# Check if uninstall was successful or if package wasn't installed
if [ $? -eq 0 ]; then
    echo "Successfully uninstalled $PACKAGE_NAME (or it was not present)."
else
    echo "Warning: Failed to uninstall $PACKAGE_NAME."
fi

echo "Step 2: Preparing source code..."
# Remove existing directory if it exists to ensure a clean install
if [ -d "$REPO_DIR" ]; then
    echo "Removing existing $REPO_DIR directory..."
    rm -rf "$REPO_DIR"
fi

# Clone the repository
echo "Cloning repository from $REPO_URL..."
git clone --recursive $REPO_URL

# Navigate into the directory
cd $REPO_DIR || { echo "Failed to enter directory $REPO_DIR"; exit 1; }

# Initialize and update submodules (redundant if --recursive is used, but good practice)
echo "Updating submodules..."
git submodule update --init --recursive

echo "Step 3: Installing $PACKAGE_NAME from source using pip..."
# Install the current directory
pip install .

# Check if install was successful
if [ $? -eq 0 ]; then
    echo "------------------------------------------------"
    echo "Success! $PACKAGE_NAME has been installed from source."
    echo "------------------------------------------------"
else
    echo "Error: Failed to install $PACKAGE_NAME."
    exit 1
fi