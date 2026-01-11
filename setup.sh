#!/bin/bash

# Define variables
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