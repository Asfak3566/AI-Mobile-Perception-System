# Multi-Orin System Setup Guide

This guide details the configuration required to synchronize three Orin devices and a central PC for data recording.

## 1. Network Configuration

**Goal**: Ensure all devices can communicate via ROS 2 over a local network (Gigabit Ethernet recommended).

### IP Address Assignment (Example)
*   **Central PC**: `192.168.6.100`
*   **Orin 1 (Sensors)**: `192.168.6.101`
*   **Orin 2 (Detection)**: `192.168.6.102`
*   **Orin 3 (Tracking)**: `192.168.6.103`

### ROS 2 Configuration
Add the following to the `~/.bashrc` file on **ALL** devices (including Central PC):

```bash
export ROS_DOMAIN_ID=42  # Must be the same on all devices
export ROS_LOCALHOST_ONLY=0
```

Apply changes:
```bash
source ~/.bashrc
```

## 2. Time Synchronization (Chrony)

**Goal**: Synchronize system clocks to millisecond precision using NTP.

### Role Assignment
*   **NTP Server (Master)**: Central PC
*   **NTP Clients**: Orin 1, Orin 2, Orin 3

### Step A: Configure Master (Central PC)
1.  Install Chrony: `sudo apt install chrony`
2.  Edit config: `sudo nano /etc/chrony/chrony.conf`
3.  Add/Uncomment these lines to allow clients:
    ```conf
    # Allow NTP client access from local network.
    allow 192.168.6.0/24

    # Serve time even if not synchronized to an external source (isolated network)
    local stratum 10
    ```
4.  Restart Chrony: `sudo systemctl restart chrony`

### Step B: Configure Clients (Orins)
1.  Install Chrony: `sudo apt install chrony`
2.  Edit config: `sudo nano /etc/chrony/chrony.conf`
3.  **Comment out** default pools (e.g., `pool 2.debian.pool.ntp.org...`).
4.  Add the Master IP:
    ```conf
    server 192.168.6.100 minpoll 0 maxpoll 2 iburst
    ```
5.  Restart Chrony: `sudo systemctl restart chrony`

### Step C: Verify Synchronization
On each Client (Orin), run:
```bash
chronyc sources
```
You should see the Master IP (`192.168.6.100`) listed with an `*` (synced) or `+` (candidate).

Run:
```bash
chronyc tracking
```
Check `System time` offset. It should be small (e.g., < 0.001 seconds).

## 3. Recording Data

Once the network and time are synced:

1.  **Start the System**: Launch the respective launch files on Orin 1, 2, and 3.
2.  **Start Recording (Central PC)**:
    ```bash
    ros2 launch mobile_mast record_all.launch.py
    ```
    *   Recordings are saved to `~/recordings/` by default.
