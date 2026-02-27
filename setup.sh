#!/bin/bash

# ==========================================
# FLS Offboard Controller Setup Script
# ==========================================

# Ensure script is run from the repo root
CURRENT_DIR=$(pwd)
echo "Running setup from: $CURRENT_DIR"

sudo -v

# ==========================================
# 1. Python Virtual Environment Setup
# ==========================================
echo "--- Python Environment Setup ---"

# Create Venv if it doesn't exist
ENV_DIR="$HOME/env"

if [ ! -d "$ENV_DIR" ]; then
    echo "Creating Python virtual environment (env)..."
    python3 -m venv "$ENV_DIR"
else
    echo "Virtual environment 'env' already exists."
fi

# Activate the environment for the rest of this script
echo "Activating virtual environment..."
source "$ENV_DIR/bin/activate"

# Add to .bashrc for auto-activation on new terminals
BASHRC_FILE="$HOME/.bashrc"
ACTIVATE_CMD="source $ENV_DIR/bin/activate"
NAVIGATE_CMD="cd $CURRENT_DIR"

echo "Checking .bashrc..."
if grep -Fq "$ACTIVATE_CMD" "$BASHRC_FILE"; then
    echo "  [OK] .bashrc already contains activation command."
else
    echo "  [UPDATE] Adding auto-activation to $BASHRC_FILE"
    echo "" >> "$BASHRC_FILE"
    echo "# FLS Offboard Controller Auto-Setup" >> "$BASHRC_FILE"
    echo "$NAVIGATE_CMD" >> "$BASHRC_FILE"
    echo "$ACTIVATE_CMD" >> "$BASHRC_FILE"
fi

echo ""

# ==========================================
# 2. System Configurations (RPI Config)
# ==========================================
echo "--- System Configuration ---"

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

if [ ! -f "$RPI_CONFIG_FILE" ]; then
    echo "Error: File $RPI_CONFIG_FILE not found."
    # We don't exit here to allow non-Pi systems to proceed with other steps,
    # but strictly speaking, this is likely a Pi-only script.
else
    echo "Checking configuration in $RPI_CONFIG_FILE..."
    for config in "${CONFIGS_TO_ADD[@]}"; do
        if grep -Fxq "$config" "$RPI_CONFIG_FILE"; then
            echo "  [OK] Already exists: $config"
        else
            echo "  [UPDATE] Adding: $config"
            echo "$config" | sudo tee -a "$RPI_CONFIG_FILE" > /dev/null
        fi
    done
fi

echo ""

# ==========================================
# 3. Network Configuration (/etc/hosts)
# ==========================================
HOSTS_FILE="/etc/hosts"
HOST_IP="192.168.1.39"
HOST_NAME="vicon"

echo "Checking $HOSTS_FILE..."
if grep -q -e "^$HOST_IP[[:space:]]\+$HOST_NAME" "$HOSTS_FILE"; then
    echo "  [OK] Entry already exists: $HOST_NAME"
else
    echo "  [UPDATE] Adding: $HOST_IP $HOST_NAME"
    printf "$HOST_IP\t$HOST_NAME\n" | sudo tee -a "$HOSTS_FILE" > /dev/null
fi

echo ""

# ==========================================
# 4. USB Permissions (Udev Rules)
# ==========================================
echo "--- Udev Rules Setup ---"

UDEV_FILE="/etc/udev/rules.d/99-crazyflie.rules"
UDEV_CONTENT='SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="0666"'

if [ -f "$UDEV_FILE" ] && grep -Fq "5740" "$UDEV_FILE"; then
     echo "  [OK] Crazyflie udev rules already exist."
else
    echo "  [UPDATE] Creating/Updating $UDEV_FILE..."
    echo "$UDEV_CONTENT" | sudo tee "$UDEV_FILE" > /dev/null

    echo "  Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

echo ""

# ==========================================
# 5. Install Dependencies
# ==========================================
echo "--- Installing Dependencies ---"

mkdir logs -p

# System dependencies
echo "Installing apt packages..."
sudo apt update && sudo apt install -y libboost-system-dev libboost-thread-dev libeigen3-dev ninja-build git

# Python dependencies (Installed into the active virtual environment)
echo "Installing Python requirements..."
pip install -r requirements.txt
pip install -r requirements_servo.txt
pip install -r requirements_led.txt

echo ""

# ==========================================
# 6. Install Libmotioncapture
# ==========================================
echo "--- Installing Libmotioncapture ---"

REPO_URL="https://github.com/Hamedamz/libmotioncapture.git"
REPO_DIR="libmotioncapture"
PACKAGE_NAME="motioncapture"

echo "Step 1: Uninstalling existing $PACKAGE_NAME library..."
pip uninstall -y $PACKAGE_NAME || true

echo "Step 2: Preparing source code..."
if [ -d "$REPO_DIR" ]; then
    echo "Removing existing $REPO_DIR directory..."
    rm -rf "$REPO_DIR"
fi

echo "Cloning repository from $REPO_URL..."
git clone --recursive $REPO_URL

# Enter directory
cd $REPO_DIR || { echo "Failed to enter directory $REPO_DIR"; exit 1; }

echo "Updating submodules..."
git submodule update --init --recursive

echo "Step 3: Installing $PACKAGE_NAME from source..."
# Since venv is active, this installs to the venv
pip install .

if [ $? -eq 0 ]; then
    echo "------------------------------------------------"
    echo "Success! $PACKAGE_NAME has been installed."
    echo "------------------------------------------------"
else
    echo "Error: Failed to install $PACKAGE_NAME."
    exit 1
fi

# Return to original directory
cd "$CURRENT_DIR"

echo "Setup Complete. Please restart your terminal or run 'source env/bin/activate' to start working."