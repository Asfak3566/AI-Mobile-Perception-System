#!/bin/bash

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit
fi

# Check for mode argument
MODE=$1
INTERFACE=$2

if [ -z "$MODE" ]; then
  echo "Usage: $0 <master|slave> [interface]"
  echo "Example: $0 slave eth0"
  exit 1
fi

if [ -z "$INTERFACE" ]; then
  INTERFACE="eth0"
  echo "No interface specified, defaulting to $INTERFACE"
fi

PRIORITY1=128
SLAVE_ONLY=0

if [ "$MODE" == "master" ]; then
  echo "Configuring as PTP MASTER on $INTERFACE..."
  PRIORITY1=127
  SLAVE_ONLY=0
elif [ "$MODE" == "slave" ]; then
  echo "Configuring as PTP SLAVE on $INTERFACE..."
  PRIORITY1=128
  SLAVE_ONLY=1
else
  echo "Invalid mode: $MODE. Use 'master' or 'slave'."
  exit 1
fi

echo "Installing linuxptp and ethtool..."
apt-get update
apt-get install -y linuxptp ethtool

echo "Creating PTP configuration /etc/linuxptp/ptp4l.conf..."
# Create a default configuration for gPTP (802.1AS) which is common in automotive
cat <<EOF > /etc/linuxptp/ptp4l.conf
[global]
gmCapable               1
priority1               ${PRIORITY1}
priority2               128
domainNumber            0
logging_level           6
use_syslog              1
verbose                 0
time_stamping           hardware
tx_timestamp_timeout    10
step_threshold          0.00002
slaveOnly               ${SLAVE_ONLY}

[$INTERFACE]
EOF

echo "Creating systemd service for ptp4l..."
cat <<EOF > /etc/systemd/system/ptp4l.service
[Unit]
Description=Precision Time Protocol (PTP) service
After=network.target

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf -i ${INTERFACE}
Restart=always

[Install]
WantedBy=multi-user.target
EOF

echo "Creating systemd service for phc2sys (syncs system clock to PTP clock)..."
cat <<EOF > /etc/systemd/system/phc2sys.service
[Unit]
Description=Synchronize system clock or PTP hardware clock (PHC)
After=ptp4l.service

[Service]
# Sync system clock (CLOCK_REALTIME) from ${INTERFACE} PHC
ExecStart=/usr/sbin/phc2sys -s ${INTERFACE} -c CLOCK_REALTIME -w
Restart=always

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd and enabling services..."
systemctl daemon-reload
systemctl enable ptp4l
systemctl enable phc2sys
systemctl restart ptp4l
systemctl restart phc2sys

echo "PTP setup complete. Check status with: systemctl status ptp4l phc2sys"
