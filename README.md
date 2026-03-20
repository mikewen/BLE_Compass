# BLE_Compass (AHRS Version)

This project implements a high-performance, tilt-compensated boat compass using BLE to receive sensor data from a QMI8658C (Accel/Gyro) and MMC5603 (Magnetometer).

## Features
- **BLE Connectivity**: Connects to the sensor node via GATT.
- **Advanced AHRS Fusion**: Uses a 1D Kalman Filter to fuse high-speed Gyroscope data (50Hz) with Magnetometer readings.
- **Tilt-Compensation**: Accurately calculates heading even when the boat is pitching or rolling.
- **Sea State Estimation**: Analyzes motion variance to provide a Sea State value (1-9) based on the Douglas Scale.
- **GPS Auto-Calibration**: Automatically aligns the magnetic heading with the GPS Course Over Ground (COG) when the boat is moving in a straight line on calm water.
- **Keep Screen On**: Toggle to prevent the phone from sleeping during navigation.
- **User-Friendly UI**: High-contrast, large-font display with real-time calibration prompts.

## Calibration
### 1. Manual Magnetometer Calibration
Click **Cal Mag** and rotate the device in all directions (including flipping it over) to capture the full 3D magnetic field. This removes Hard-Iron and Soft-Iron biases.

### 2. Manual Gyroscope Calibration
Click **Cal Gyro** while the device is perfectly still to remove the zero-rate bias.

### 3. GPS Auto-Calibration
When **Sea State <= 2** and **Boat Speed > 2.0 kn** (configurable), the app will slowly adjust the compass offset to match your GPS track. This accounts for local magnetic deviation and sensor mounting alignment.

## UI Layout
- **Heading**: Large red number (0-360°).
- **Sea State**: Green indicator of water agitation.
- **GPS Info**: Shows current speed and the active GPS-derived offset.
- **Screen Toggle**: Green (On) / Grey (Off) button for display persistence.

## Project Structure
- `MainActivity.kt`: Core fusion logic, GPS tracking, and UI management.
- `ScanActivity.kt`: BLE device discovery.
- `ic_compass_app.xml`: Custom vector app icon.
