# BLE Compass (High-Precision AHRS)

A professional-grade boat compass that uses BLE to receive raw 9-axis data from a QMI8658 (Accel/Gyro) and MMC5603 (Magnetometer). 

## 🚀 Key Features
- **NED Fusion Logic**: Implements industry-standard North-East-Down coordinate system for stable 360° heading.
- **Kalman Filter**: Fuses 50Hz Gyroscope data with Magnetometer readings to eliminate lag and vibration noise.
- **High-Precision Tilt Compensation**: Uses a full rotation matrix to project magnetic vectors onto the horizontal plane, keeping the heading accurate during pitch and roll.
- **GPS Auto-Calibration**: Automatically calculates the magnetic deviation and mounting offset by comparing the compass to the GPS Course Over Ground (COG) when the boat is moving (>2kn) in calm water.
- **Sea State Monitoring**: Real-time analysis of vessel agitation using variance-based motion detection (1-9 scale).
- **Raw Data Logger**: Automatically logs sensor data to `.txt` for advanced debugging and post-processing.

## 🛠 Hardware Alignment
The code is pre-configured for modules where the Magnetometer is rotated 180° relative to the Accelerometer.
- `ACCT_ROTATED_180 = true`: Aligns Accel/Gyro to the Mag frame.
- `INVERT_MAG_Z = true`: Corrects for Z-axis polarity mismatch.
- `INVERT_GYRO_Z = true`: Aligns Gyro rotation with Clockwise compass heading.

## 📐 Calibration Guide
1. **Cal Mag**: Rotate the device in a figure-8 pattern. **CRITICAL:** You must flip the device completely upside down several times to calibrate the Z-axis. This fixes tilt-up/down errors.
2. **Cal Gyro**: Keep the device perfectly still for 5 seconds (countdown provided).
3. **Auto-Cal**: Drive the boat in a straight line on calm water. The app will gradually align the magnetic heading to your True GPS heading.

## 📂 Developer Notes
- **Fusion Frequency**: 50Hz (20ms packets).
- **UI Refresh**: 5Hz (200ms) for readability.
- **Data Log**: Found in `/Android/data/com.mikewen.ble_compass/files/raw_sensor_data.txt`.
