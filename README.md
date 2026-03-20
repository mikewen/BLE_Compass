# BLE_Compass (AHRS Version)

This project implements a tilt-compensated compass using BLE to receive sensor data from a QMI8658C (Accel/Gyro) and MMC5603 (Magnetometer).

## Features
- **BLE Scanning & Connection**: Easily find and connect to your sensor node.
- **Real-time Data Parsing**: Handles binary data packets containing Accelerometer, Gyroscope, and Magnetometer readings.
- **Tilt-Compensation**: Uses accelerometer data (Roll/Pitch) to correct magnetometer readings, providing an accurate heading even when the device isn't level.
- **Sensor Calibration**: 
    - **Magnetometer**: Hard-iron and soft-iron compensation by rotating the device.
    - **Gyroscope**: Bias removal (Zero-rate offset) by keeping the device still.
- **Smoothing**: Low-pass filtering for stable heading output.

## Calibration
Calibration data is persisted across app restarts using **SharedPreferences** (stored in `calib_prefs.xml`).

### How to Calibrate:
1. Tap **Start Calibration**.
2. **Gyroscope**: Keep the device perfectly still for a few seconds to calculate the zero-bias offset.
3. **Magnetometer**: Rotate the device slowly in a "figure-8" or level circle to capture the full range of the magnetic field.
4. Tap **Finish Calibration** to save the offsets and scales.

## Technical Details
- **QMI8658C**: 6-axis IMU providing Acceleration and Gyroscope data.
- **MMC5603**: 3-axis Magnetometer.
- **Communication**: BLE GATT notifications on Service `0000ae30-...` and Characteristic `0000ae02-...`.

## Recent Fixes
- Fixed `Class is not allowed here` error in `ScanActivity.kt` by removing unnecessary `inner` modifier.
- Integrated persistent calibration storage for both Mag and Gyro.
