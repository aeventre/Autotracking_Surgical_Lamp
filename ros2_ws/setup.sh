#!/bin/bash

echo "Setting up surgical lamp virtual environment..."

# Step 1: Create virtual environment if it doesn't exist
if [ ! -d "environment" ]; then
    echo "Creating virtual environment..."
    python3 -m venv environment
fi

# Step 2: Activate the virtual environment
echo "Activating virtual environment..."
source environment/bin/activate

# Step 3: Install Python dependencies
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies..."
    pip install -r requirements.txt
else
    echo "No requirements.txt found. Skipping Python dependency installation."
fi

# Step 4: Clear lingering ROS2 environment variables
unset ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH

# Step 5: Source ROS2 workspace
if [ -f "ros2_ws/install/setup.bash" ]; then
    echo "Sourcing ROS2 workspace..."
    source ros2_ws/install/setup.bash
else
    echo "ROS2 workspace not built yet. Run 'colcon build' in ros2_ws first."
fi

echo "Environment setup complete."