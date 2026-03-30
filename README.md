# BLE Navigation Hub (Precision IMU + GPS)

A high-performance Android application designed for marine navigation, fusing 9-axis IMU data with high-precision BLE GPS orientation and position data.

## 🚀 Key Features
- **Dual BLE Connectivity**: Simultaneously connect to a QMI8658+MMC5603 IMU module and an 'AC6328_GPS' sensor.
- **Smart GPS Switching**: 
    - **Phone GPS Fallback**: The internal phone GPS is only used if the external BLE GPS is disconnected OR if no valid position data (`0xA3`) has been received from the BLE sensor for more than 5 seconds.
    - **Precision Position**: Uses high-accuracy speed and Course Over Ground (COG) from the `0xA3` position packets.
    - **Precision Orientation**: Leverages `0xA2` orientation packets; if `quality == 4`, the GPS-derived heading updates the Kalman filter immediately at 1Hz.
- **Multi-rate Kalman Filter**: A robust fusion engine optimized for marine dynamics:
    - **Predict (Gyro) @ 50Hz**: Integrated for instantaneous response.
    - **Magnetometer Update @ 10Hz**: Provides absolute reference with speed-dependent weighting.
    - **GPS/COG Update @ 1Hz**: Provides true ground reference for drift correction.
- **NXP Standard Tilt Compensation**: Integrated within the filter to maintain heading accuracy during significant vessel pitch and roll.
- **Sea State Monitoring**: Real-time analysis of vessel agitation using variance-based motion detection (1-100 scale).
- **Orientation Warnings**: Real-time Pitch and Roll monitoring with a Red-text visual warning if Pitch > 5° or Roll > 15°.
- **Autopilot PID Controller**: Integrated PID loop for heading maintenance with boat-optimized sea-state deadbanding.

## 🛠 Hardware Alignment
The code is pre-configured for modules where the Magnetometer is rotated 180° relative to the Accelerometer.
- `ACCT_ROTATED_180 = true`: Aligns Accel/Gyro to the Mag frame.
- `INVERT_MAG_Z = true`: Corrects for Z-axis polarity mismatch.
- `INVERT_GYRO_Z = true`: Aligns Gyro rotation with Clockwise compass heading.

## 📐 Calibration
1. **Cal Mag**: Rotate the device in a figure-8 pattern, ensuring several 360° flips to calibrate the Z-axis.
2. **Cal Gyro**: Keep the device perfectly still for 5 seconds to eliminate bias.
3. **True Heading Sync**: The app automatically aligns the IMU mounting offset by comparing it to GPS Course Over Ground (COG) when moving > 2kn.

## 📈 Filter Logic & Speed Dependencies
For high-performance marine use, the filter dynamically adjusts its trust in sensors based on boat speed:
- **GPS COG Updates**: Course Over Ground is only used to update the Kalman heading if `boatSpeed > 0.8 m/s`.
- **Magnetometer Weighting**: Magnetometer trust is reduced (`kfR` increases) when `boatSpeed > 2.0 m/s` to prioritize stable GPS/Gyro heading data at cruising speeds.
- **Immediate GPS Correction**: High-quality GPS ORI (`0xA2`, quality 4) updates the filter regardless of speed.

## 📝 How to Make Simple Changes
- **Change Sensor Orientation**: Locate the `Hardware Alignment` section at the top of `MainActivity.kt` and toggle the boolean flags.
- **Adjust Filter Tuning (Kalman)**: 
    - **Increase Gyro Weight (Short-term stability)**: To trust the gyroscope integration more and the magnetometer less, **increase `kfR`** (e.g., from 0.1 to 0.5) or **decrease `kfQ`** (e.g., from 0.0005 to 0.0001). This reduces heading "jitter" but makes the compass slower to correct long-term drift.
    - **Faster Response**: Increase `kfQ` or decrease `kfR`.
- **Modify Warning Thresholds**: Search for `updateUi` in `MainActivity.kt` and change the `abs(pitch) > 5.0` or `abs(roll) > 15.0` values.
- **Custom Packet Parsing**: Add new `case` statements inside the `when(header)` block in `processRawData()`.

## 📉 Advanced: Dynamic Weighting
For high-performance marine use, consider scaling `kfR` based on `currentSeaState`. When the boat is tossing (High Sea State), the magnetometer tilt-compensation becomes less reliable. In these moments, you can dynamically increase `kfR` inside `processRawData()` to rely almost entirely on the gyroscope until the motion subsides.

## 📂 Code Structure
- **`MainActivity.kt`**: The core engine. Handles BLE life-cycle, sensor fusion, and UI dispatching.
    - `processRawData()`: The central dispatcher for incoming BLE bytes (`0xA1`, `0xA2`, `0xA3`).
    - `runKalmanPredict()` / `runKalmanUpdate()`: The mathematical heart. Performs sensor fusion at varying rates.
    - `updateUi()`: Manages display updates and orientation warning colors.
- **`ScanActivity.kt`**: BLE scanner implementation.

## 📁 Developer Notes
- **Rates**: Gyro 50Hz | Mag 10Hz | GPS 1Hz.
- **UI Refresh**: 20Hz (approx. 50ms interval).
- **Coordinate System**: NED (North-East-Down).
- **Log Data**: Found in `/Android/data/com.mikewen.ble_compass/files/raw_sensor_data.txt`.
