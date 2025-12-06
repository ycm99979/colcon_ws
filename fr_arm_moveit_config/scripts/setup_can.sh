#!/bin/bash
# CAN Interface Setup Script for MyActuator RMD Motors
# Usage: sudo ./setup_can.sh [interface] [bitrate]
#
# Example:
#   sudo ./setup_can.sh can0 1000000
#   sudo ./setup_can.sh can1 500000

CAN_INTERFACE=${1:-can0}
BITRATE=${2:-1000000}

echo "============================================"
echo "Setting up CAN interface: $CAN_INTERFACE"
echo "Bitrate: $BITRATE bps"
echo "============================================"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: Please run as root (sudo)"
    exit 1
fi

# Check if interface exists
if ! ip link show $CAN_INTERFACE > /dev/null 2>&1; then
    echo "Error: Interface $CAN_INTERFACE not found!"
    echo ""
    echo "Available network interfaces:"
    ip link show | grep -E "^[0-9]+:" | awk '{print $2}' | tr -d ':'
    echo ""
    echo "If using USB-CAN adapter, check if it's connected."
    echo "You may need to load the can module: modprobe can"
    exit 1
fi

# Bring interface down first
echo "Bringing down $CAN_INTERFACE..."
ip link set $CAN_INTERFACE down 2>/dev/null

# Set CAN type and bitrate
echo "Configuring CAN type and bitrate..."
if ! ip link set $CAN_INTERFACE type can bitrate $BITRATE; then
    echo "Error: Failed to set CAN parameters"
    exit 1
fi

# Bring interface up
echo "Bringing up $CAN_INTERFACE..."
if ! ip link set $CAN_INTERFACE up; then
    echo "Error: Failed to bring up interface"
    exit 1
fi

# Verify
echo ""
echo "============================================"
echo "CAN Interface Status:"
echo "============================================"
ip -details link show $CAN_INTERFACE

echo ""
echo "============================================"
echo "Setup complete! You can now run:"
echo "  ros2 launch fr_arm_moveit_config hardware.launch.py can_interface:=$CAN_INTERFACE"
echo "============================================"

# Optional: Monitor CAN traffic
echo ""
echo "To monitor CAN traffic, run in another terminal:"
echo "  candump $CAN_INTERFACE"
