# system-health-monitor

This is a basic system that allows for checking the health of a system by monitoring various metrics such as CPU usage, memory usage, disk space, and network activity.

Will also check things like sensor publishing rates, and if the system is publishing data to a database.

## Features

Computer Info

[ ] Monitor CPU usage
[ ] Monitor memory usage
[ ] Monitor disk space
[ ] Monitor network activity
[ ] ???

Camera Info

[ ] Monitor camera publishing rates
[ ] Blur detection in camera images
[ ] Check similarity between stereo images
[ ] ???

LIDAR Info

[ ] Monitor LIDAR publishing rates
[ ] Check that scans are fully populated
[ ] ???

IMU Info

[ ] Monitor IMU publishing rates
[ ] Check consistency of magnetometer
[ ] ???


Radio Info

[ ] Monitor radio publishing rates
[ ] Check how often an iperf3 test throws an error
[ ] ???



# ChatGPT Suggestions

Here's a detailed and expanded checklist with important additions in `[ ] ???` slots based on best practices for system and sensor health monitoring in robotics, especially for field-deployable autonomous systems like yours:

---

### **Computer Info**

* [x] Monitor CPU usage
* [x] Monitor memory usage
* [x] Monitor disk space
* [x] Monitor network activity
* [ ] **Check system clock synchronization (e.g. NTP drift)**
* [ ] **Monitor GPU usage (if applicable, e.g. Jetson/RTX devices)**
* [ ] **Track process uptime / watchdog timeouts (self-diagnostics)**
* [ ] **Thermal monitoring (CPU/GPU temperature thresholds)**
* [ ] **ROS node liveness (e.g. missed heartbeats or stale publishers)**

---

### **Camera Info**

* [x] Monitor camera publishing rates
* [x] Blur detection in camera images
* [x] Check similarity between stereo images
* [ ] **Check for frozen images (no content change over time)**
* [ ] **Overexposure/underexposure check (saturation, dynamic range)**
* [ ] **Check resolution and encoding correctness**
* [ ] **Verify that `camera_info` is valid and matches image resolution**
* [ ] **Latency between capture time and ROS timestamp**

---

### **LIDAR Info**

* [x] Monitor LIDAR publishing rates
* [x] Check that scans are fully populated
* [ ] **Check for consistent angular resolution and field of view**
* [ ] **Validate timestamp spacing between packets or scans**
* [ ] **Compare range data statistics (mean/max) to detect stale or noisy scans**
* [ ] **Detect low return rates (e.g., due to fog or occlusion)**
* [ ] **Confirm extrinsics (TF from base to LIDAR) is being published and valid**

---

### **IMU Info**

* [x] Monitor IMU publishing rates
* [x] Check consistency of magnetometer
* [ ] **Detect stuck or saturated accelerometer/gyro readings**
* [ ] **Check noise levels (standard deviation over time)**
* [ ] **Verify unit and range (e.g., ±16g, rad/s limits)**
* [ ] **Monitor IMU orientation drift (if fused with filter)**
* [ ] **Cross-check gravity alignment (accelerometer vs expected down vector)**

---

### **Radio Info**

* [x] Monitor radio publishing rates
* [x] Check how often an iperf3 test throws an error
* [ ] **Track signal strength (RSSI/SNR if available from driver)**
* [ ] **Monitor packet drop/loss rate over ROS topics or UDP transport**
* [ ] **Evaluate bandwidth consistency (min/avg/max throughput)**
* [ ] **Check link stability (e.g., periodic disconnection events)**
* [ ] **Latency of radio interface (ping RTT, jitter)**
* [ ] **Verify IP routing or NAT traversal if using mesh/VPN**
