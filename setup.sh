#!/usr/bin/env bash
# One-time host setup: ROS 2 Humble packages + RealSense udev rules.
# Run with: sudo ./setup.sh   (then unplug/replug the D455)
set -euo pipefail
cd "$(dirname "$0")"

if [[ $EUID -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 1
fi

pkgs=(ros-humble-realsense2-camera ros-humble-rtabmap-ros ros-humble-imu-filter-madgwick)
missing=()
for p in "${pkgs[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
done
if (( ${#missing[@]} )); then
  apt-get update
  apt-get install -y "${missing[@]}"
fi

# Without these rules the D455 IMU (HID/IIO sysfs nodes) is root-only and
# realsense2_camera fails with "Failed to open scan_element" / "Hid device is busy".
install -m 0644 udev/99-realsense-libusb.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

echo "Done. Unplug and replug the D455, then run ./run.sh"
