# BLE Navigation Hub (Precision IMU + GPS)

A high-performance Android application designed for marine navigation, fusing 9-axis IMU data with high-precision BLE GPS orientation and position data.

## 🚀 Key Features
- **Dual BLE Connectivity**: Simultaneously connect to a QMI8658+MMC5603 IMU module and an 'AC6328_GPS' sensor.
- **Smart Switching**: 
    - Priority-based display: Uses IMU Kalman fusion by default.
    - Automatic fallback: Switches to BLE GPS orientation or course data if the IMU disconnects.
    - Phone GPS fallback: Automatically uses internal phone GPS if BLE GPS is unavailable.
- **Kalman Filter (1D)**: Fuses Gyroscope and Magnetometer data to provide a lag-free, vibration-resistant 360° heading.
- **NXP Standard Tilt Compensation**: Integrated within the Kalman filter to maintain heading accuracy during significant vessel pitch and roll.
- **Orientation Warnings**: Real-time Pitch and Roll monitoring with a Red-text visual warning if Pitch > 5° or Roll > 15°.
- **Autopilot PID Controller**: Integrated PID loop for heading maintenance with boat-optimized sea-state deadbanding.
- **Live Raw Data Monitoring**: A dedicated debug field at the bottom shows real-time parsed packets from the active sensor.

## 🛠 Sensor Packet Specification
The app expects the following packet structures over BLE notifications:
- **0xA1 (IMU)**: 20 bytes - Accel[X,Y,Z], Gyro[X,Y,Z], Mag[X,Y,Z] (Little Endian).
- **0xA2 (GPS ORI)**: 17 bytes - Time, Pitch, Roll, Heading, Accuracy (Little Endian).
- **0xA3 (GPS POS)**: 17 bytes - Time, Latitude, Longitude, Speed, Course (Little Endian).

## 📐 Calibration
1. **Cal Mag**: Rotate the device in a figure-8 pattern, ensuring several 360° flips to calibrate the Z-axis.
2. **Cal Gyro**: Keep the device perfectly still for 5 seconds to eliminate bias.
3. **True Heading Sync**: The app automatically aligns the IMU mounting offset by comparing it to GPS Course Over Ground (COG) when moving > 2kn.

## 📂 Developer Notes
- **Fusion Rate**: 50Hz (20ms packets).
- **UI Refresh**: 5Hz (200ms).
- **Coordinate System**: NED (North-East-Down).
