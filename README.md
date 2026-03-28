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
- **Sea State Monitoring**: Real-time analysis of vessel agitation using variance-based motion detection (1-100 scale).
- **Orientation Warnings**: Real-time Pitch and Roll monitoring with a Red-text visual warning if Pitch > 5° or Roll > 15°.
- **Autopilot PID Controller**: Integrated PID loop for heading maintenance with boat-optimized sea-state deadbanding.
- **Live Raw Data Monitoring**: A dedicated debug field at the bottom shows real-time parsed packets from the active sensor.

## 🛠 Hardware Alignment
The code is pre-configured for modules where the Magnetometer is rotated 180° relative to the Accelerometer.
- `ACCT_ROTATED_180 = true`: Aligns Accel/Gyro to the Mag frame.
- `INVERT_MAG_Z = true`: Corrects for Z-axis polarity mismatch.
- `INVERT_GYRO_Z = true`: Aligns Gyro rotation with Clockwise compass heading.

## 📐 Calibration
1. **Cal Mag**: Rotate the device in a figure-8 pattern, ensuring several 360° flips to calibrate the Z-axis.
2. **Cal Gyro**: Keep the device perfectly still for 5 seconds to eliminate bias.
3. **True Heading Sync**: The app automatically aligns the IMU mounting offset by comparing it to GPS Course Over Ground (COG) when moving > 2kn.

## 📂 Code Structure
- **`MainActivity.kt`**: The core engine. Handles BLE life-cycle, sensor fusion, and UI dispatching.
    - `processRawData()`: The central dispatcher for incoming BLE bytes (`0xA1`, `0xA2`, `0xA3`).
    - `runKalman()`: The mathematical heart. Performs tilt compensation and sensor fusion.
    - `updateUi()`: Manages the high-speed display updates and orientation warning colors.
- **`ScanActivity.kt`**: BLE scanner implementation for finding and connecting to your hardware.
- **`activity_main.xml`**: Layout defining the prioritized data views and Autopilot controls.

## 📝 How to Make Simple Changes
- **Change Sensor Orientation**: Locate the `Hardware Alignment` section at the top of `MainActivity.kt` and toggle the boolean flags.
- **Adjust Filter Tuning (Kalman)**: 
    - **Increase Gyro Weight (Short-term stability)**: To trust the gyroscope integration more and the magnetometer less, **increase `kfR`** (e.g., from 0.1 to 0.5) or **decrease `kfQ`** (e.g., from 0.0005 to 0.0001). This reduces heading "jitter" but makes the compass slower to correct long-term drift.
    - **Faster Response**: Increase `kfQ` or decrease `kfR`.
- **Modify Warning Thresholds**: Search for `updateUi` in `MainActivity.kt` and change the `abs(pitch) > 5.0` or `abs(roll) > 15.0` values.
- **Custom Packet Parsing**: Add new `case` statements inside the `when(header)` block in `processRawData()`.

## 📈 Advanced: Dynamic Weighting
For high-performance marine use, consider scaling `kfR` based on `currentSeaState`. When the boat is tossing (High Sea State), the magnetometer tilt-compensation becomes less reliable. In these moments, you can dynamically increase `kfR` inside `processRawData()` to rely almost entirely on the gyroscope until the motion subsides.

## 📁 Developer Notes
- **Fusion Rate**: 50Hz (20ms packets).
- **UI Refresh**: 5Hz (200ms).
- **Coordinate System**: NED (North-East-Down).
- **Log Data**: Found in `/Android/data/com.mikewen.ble_compass/files/raw_sensor_data.txt`.
