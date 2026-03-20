package com.mikewen.ble_compass

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.*
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.os.*
import android.view.View
import android.view.WindowManager
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.mikewen.ble_compass.databinding.ActivityMainBinding
import java.util.*
import kotlin.math.*

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var bluetoothGatt: BluetoothGatt? = null
    private lateinit var prefs: SharedPreferences

    private val SERVICE_UUID = UUID.fromString("0000ae30-0000-1000-8000-00805f9b34fb")
    private val CHAR_UUID = UUID.fromString("0000ae02-0000-1000-8000-00805f9b34fb")

    // Magnetometer Calibration
    private var isCalibratingMag = false
    private var minMx = Double.MAX_VALUE; private var maxMx = Double.MIN_VALUE
    private var minMy = Double.MAX_VALUE; private var maxMy = Double.MIN_VALUE
    private var minMz = Double.MAX_VALUE; private var maxMz = Double.MIN_VALUE
    
    private var offsetX = 0.0; private var offsetY = 0.0; private var offsetZ = 0.0
    private var scaleX = 1.0; private var scaleY = 1.0

    // Gyroscope Calibration
    private var isCalibratingGyro = false
    private var gyroOffsetZ = 0.0
    private var gyroSumZ = 0.0
    private var gyroCount = 0

    // Kalman Filter Variables
    private var kfHeading = 0.0
    private var kfP = 1.0
    private val kfQ = 0.0005    // Trust gyro more for transients
    private val kfR = 0.1       // Measurement noise
    private var lastTimestamp = 0L
    private var lastUiUpdateTime = 0L

    // Sea State variables
    private val motionBuffer = mutableListOf<Double>()
    private val BUFFER_SIZE = 50 // 1 second at 50Hz
    private var currentSeaState = 1

    private var isKeepScreenOn = false

    private val scanLauncher = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
        if (result.resultCode == Activity.RESULT_OK) {
            val address = result.data?.getStringExtra("device_address")
            if (address != null) connectToDevice(address)
        }
    }

    private val requestPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            if (permissions.entries.all { it.value }) openScanActivity()
            else Toast.makeText(this, "Permissions required", Toast.LENGTH_SHORT).show()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        prefs = getSharedPreferences("calib_prefs", Context.MODE_PRIVATE)
        loadCalibration()

        binding.btnScan.setOnClickListener { checkPermissionsAndScan() }
        binding.btnCalibrateMag.setOnClickListener { if (!isCalibratingMag) startMagCalibration() else stopMagCalibration() }
        binding.btnCalibrateGyro.setOnClickListener { if (!isCalibratingGyro) startGyroCalibration() else stopGyroCalibration() }

        binding.btnKeepScreenOn.setOnClickListener {
            isKeepScreenOn = !isKeepScreenOn
            if (isKeepScreenOn) {
                window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                binding.btnKeepScreenOn.text = "Keep Screen On: ON"
                binding.btnKeepScreenOn.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#4CAF50"))
            } else {
                window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                binding.btnKeepScreenOn.text = "Keep Screen On: OFF"
                binding.btnKeepScreenOn.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#607D8B"))
            }
        }
    }

    private fun loadCalibration() {
        offsetX = prefs.getFloat("mag_off_x", 0.0f).toDouble()
        offsetY = prefs.getFloat("mag_off_y", 0.0f).toDouble()
        offsetZ = prefs.getFloat("mag_off_z", 0.0f).toDouble()
        scaleX = prefs.getFloat("mag_scale_x", 1.0f).toDouble()
        scaleY = prefs.getFloat("mag_scale_y", 1.0f).toDouble()
        gyroOffsetZ = prefs.getFloat("gyro_off_z", 0.0f).toDouble()
    }

    private fun saveCalibration() {
        prefs.edit().apply {
            putFloat("mag_off_x", offsetX.toFloat()); putFloat("mag_off_y", offsetY.toFloat()); putFloat("mag_off_z", offsetZ.toFloat())
            putFloat("mag_scale_x", scaleX.toFloat()); putFloat("mag_scale_y", scaleY.toFloat())
            putFloat("gyro_off_z", gyroOffsetZ.toFloat())
            apply()
        }
    }

    private fun checkPermissionsAndScan() {
        val permissions = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)

        val missing = permissions.filter { ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isEmpty()) openScanActivity() else requestPermissionLauncher.launch(missing.toTypedArray())
    }

    private fun openScanActivity() { scanLauncher.launch(Intent(this, ScanActivity::class.java)) }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(address: String) {
        val device = (getSystemService(BLUETOOTH_SERVICE) as BluetoothManager).adapter.getRemoteDevice(address)
        binding.txtStatus.text = "Status: Connecting..."
        bluetoothGatt = device.connectGatt(this, false, gattCallback)
    }

    private fun startMagCalibration() {
        isCalibratingMag = true
        minMx = 1e6; maxMx = -1e6; minMy = 1e6; maxMy = -1e6; minMz = 1e6; maxMz = -1e6
        binding.btnCalibrateMag.text = "STOP MAG CALIBRATION"
        binding.txtCalibStatus.text = "Rotate in ALL directions (including flipping it!)"
        binding.txtCalibStatus.visibility = View.VISIBLE
    }

    private fun stopMagCalibration() {
        isCalibratingMag = false
        binding.btnCalibrateMag.text = "Calibrate Magnetometer"
        binding.txtCalibStatus.visibility = View.GONE
        if (maxMx > minMx && maxMy > minMy) {
            offsetX = (maxMx + minMx) / 2.0; offsetY = (maxMy + minMy) / 2.0
            if (maxMz > minMz) offsetZ = (maxMz + minMz) / 2.0
            val avgDeltaX = (maxMx - minMx) / 2.0; val avgDeltaY = (maxMy - minMy) / 2.0
            val avgDelta = (avgDeltaX + avgDeltaY) / 2.0
            scaleX = if (avgDeltaX > 1.0) avgDelta / avgDeltaX else 1.0
            scaleY = if (avgDeltaY > 1.0) avgDelta / avgDeltaY else 1.0
            saveCalibration()
            Toast.makeText(this, "Mag Calibration Saved", Toast.LENGTH_SHORT).show()
        }
    }

    private fun startGyroCalibration() {
        isCalibratingGyro = true; gyroSumZ = 0.0; gyroCount = 0
        binding.btnCalibrateGyro.text = "STOP GYRO CALIBRATION"
        binding.txtCalibStatus.text = "Keep device PERFECTLY STILL..."
        binding.txtCalibStatus.visibility = View.VISIBLE
    }

    private fun stopGyroCalibration() {
        isCalibratingGyro = false; binding.btnCalibrateGyro.text = "Calibrate Gyroscope"; binding.txtCalibStatus.visibility = View.GONE
        if (gyroCount > 0) {
            gyroOffsetZ = gyroSumZ / gyroCount
            saveCalibration()
            Toast.makeText(this, "Gyro Calibration Saved", Toast.LENGTH_SHORT).show()
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            runOnUiThread {
                if (newState == BluetoothProfile.STATE_CONNECTED) { 
                    binding.txtStatus.text = "Status: Connected"
                    binding.txtStatus.setTextColor(0xFF4CAF50.toInt()) // Green
                    gatt.discoverServices() 
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    binding.txtStatus.text = "Status: Disconnected"
                    binding.txtStatus.setTextColor(0xFFF44336.toInt()) // Red
                }
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            val service = gatt.getService(SERVICE_UUID)
            val char = service?.getCharacteristic(CHAR_UUID)
            if (char != null) {
                gatt.setCharacteristicNotification(char, true)
                val descriptor = char.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
                descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                gatt.writeDescriptor(descriptor)
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, ch: BluetoothGattCharacteristic) {
            val d = ch.value
            if (d.size >= 20 && d[0] == 0xA1.toByte()) {
                val now = System.currentTimeMillis()
                val dt = if (lastTimestamp == 0L) 0.02 else (now - lastTimestamp) / 1000.0
                lastTimestamp = now

                val ax = ((d[3].toInt() shl 8) or (d[2].toInt() and 0xFF)).toShort().toDouble()
                val ay = ((d[5].toInt() shl 8) or (d[4].toInt() and 0xFF)).toShort().toDouble()
                val az = ((d[7].toInt() shl 8) or (d[6].toInt() and 0xFF)).toShort().toDouble()
                val gxRaw = ((d[9].toInt() shl 8) or (d[8].toInt() and 0xFF)).toShort().toDouble()
                val gyRaw = ((d[11].toInt() shl 8) or (d[10].toInt() and 0xFF)).toShort().toDouble()
                val gzRaw = ((d[13].toInt() shl 8) or (d[12].toInt() and 0xFF)).toShort().toDouble()
                val mxRaw = ((d[15].toInt() shl 8) or (d[14].toInt() and 0xFF)).toShort().toDouble()
                val myRaw = ((d[17].toInt() shl 8) or (d[16].toInt() and 0xFF)).toShort().toDouble()
                val mzRaw = ((d[19].toInt() shl 8) or (d[18].toInt() and 0xFF)).toShort().toDouble()

                if (isCalibratingMag) {
                    minMx = min(minMx, mxRaw); maxMx = max(maxMx, mxRaw)
                    minMy = min(minMy, myRaw); maxMy = max(maxMy, myRaw)
                    minMz = min(minMz, mzRaw); maxMz = max(maxMz, mzRaw)
                }
                if (isCalibratingGyro) { gyroSumZ += gzRaw; gyroCount++ }

                // 1. Apply Calibration
                val mx = (mxRaw - offsetX) * scaleX
                val my = (myRaw - offsetY) * scaleY
                val mz = (mzRaw - offsetZ)
                val gzDeg = -(gzRaw - gyroOffsetZ) / 64.0 

                // 2. Calculate Tilt (Roll/Pitch)
                val roll = atan2(ay, az)
                val pitch = atan2(-ax, sqrt(ay * ay + az * az))

                // 3. Project Magnetometer to Horizontal Plane
                val xh = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch)
                val yh = my * cos(roll) - mz * sin(roll)

                // 4. Calculate Mag Heading
                val magHeading = atan2(yh, xh) 

                // 5. Kalman Fusion
                kfHeading += Math.toRadians(gzDeg * dt) 
                kfP += kfQ

                var angleDiff = magHeading - kfHeading
                while (angleDiff > PI) angleDiff -= 2 * PI
                while (angleDiff < -PI) angleDiff += 2 * PI

                val kGain = kfP / (kfP + kfR)
                kfHeading += kGain * angleDiff
                
                while (kfHeading > PI) kfHeading -= 2 * PI
                while (kfHeading < -PI) kfHeading += 2 * PI

                // Sea State calculation (Variance based)
                val motionMagnitude = sqrt(ax*ax + ay*ay + az*az) / 16384.0 + 
                                     sqrt(gxRaw*gxRaw + gyRaw*gyRaw + gzRaw*gzRaw) / 32768.0
                motionBuffer.add(motionMagnitude)
                if (motionBuffer.size > BUFFER_SIZE) motionBuffer.removeAt(0)
                
                val avgMotion = motionBuffer.average()
                val variance = motionBuffer.map { (it - avgMotion).pow(2) }.average()
                currentSeaState = when {
                    variance < 0.0001 -> 1
                    variance < 0.001 -> 2
                    variance < 0.005 -> 3
                    variance < 0.01 -> 4
                    variance < 0.05 -> 5
                    variance < 0.1 -> 6
                    variance < 0.2 -> 7
                    variance < 0.5 -> 8
                    else -> 9
                }

                // 6. Throttled UI Update (every 200ms)
                if (now - lastUiUpdateTime > 200) {
                    lastUiUpdateTime = now
                    var deg = Math.toDegrees(kfHeading)
                    if (deg < 0) deg += 360.0

                    runOnUiThread {
                        binding.txtHeading.text = "%.0f°".format(deg)
                        binding.txtStatus.text = "X:%.0f Y:%.0f Z:%.0f".format(mx, my, mz)
                        binding.txtSeaState.text = "Sea State: $currentSeaState"
                    }
                }
            }
        }
    }
}
