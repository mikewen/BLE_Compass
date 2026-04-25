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
import android.hardware.GeomagneticField
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.*
import android.text.Editable
import android.text.TextWatcher
import android.util.Log
import android.view.View
import android.view.WindowManager
import android.widget.*
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.mikewen.ble_compass.databinding.ActivityMainBinding
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.*
import kotlin.math.*

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var imuGatt: BluetoothGatt? = null
    private var gpsGatt: BluetoothGatt? = null
    private lateinit var prefs: SharedPreferences
    private lateinit var locationManager: LocationManager

    // --- Hardware Alignment Configuration ---
    private val ACCT_ROTATED_180 = true  // Aligns IMU to Mag PCB orientation
    private var INVERT_MAG_Z = true      // Set true if Pitch Up behaves opposite to Pitch Down
    private var INVERT_GYRO_Z = true     // Standard compass: CW is positive, so flip CCW gyro
    // ----------------------------------------

    private var isConnected = false
    private var isGpsConnected = false
    private var displayMode = 0 // 0: IMU, 1: GPS

    // Calibration state
    private var minMx = 1e6; private var maxMx = -1e6; private var minMy = 1e6; private var maxMy = -1e6; private var minMz = 1e6; private var maxMz = -1e6
    private var offsetX = 0.0; private var offsetY = 0.0; private var offsetZ = 0.0
    private var scaleX = 1.0; private var scaleY = 1.0; private var scaleZ = 1.0
    private var isCalibratingMag = false
    private var isCalibratingGyro = false
    private var gyroOffX = 0.0; private var gyroOffY = 0.0; private var gyroOffZ = 0.0
    private var gyroSumX = 0.0; private var gyroSumY = 0.0; private var gyroSumZ = 0.0

    var gyroScaleDegS: Float = 256f
    private var gyroCount = 0
    private var gyroCalibTimer: CountDownTimer? = null
    private var calDialog: AlertDialog? = null

    // AHRS State (Multi-rate Kalman)
    private var currentHeading = 0.0
    private var kfHeading = 0.0 // radians
    private var kfP = 1.0
    private val kfQ = 0.0005
    private val kfR_mag_normal = 0.1
    private val kfR_mag_low_weight = 0.8
    private val kfR_gps = 0.05
    private var lastMagUpdateTime = 0L
    private var lastGpsKalmanUpdateTime = 0L
    private var lastA3Timestamp = 0L

    // Declination handling (Persistent)
    private var magneticDeclination = 0.0f
    private var lastDeclinationUpdate = 0L

    // Sea State
    private val motionBuffer = mutableListOf<Double>()
    private val BUFFER_SIZE = 50 
    private var currentSeaState = 1

    // Autopilot PID
    private var PID_INTERVAL_MS = 200L 
    private var targetHeading: Double? = null 
    private var pidKp = 0.8; private var pidKi = 0.02; private var pidKd = 0.5
    private var pidIntegral = 0.0; private var pidLastError = 0.0
    private var deadband = 3.0; private var isAutoDeadband = false
    private var escThrottle = 580.0
    private var bldcThrottle = 2000.0
    
    private val pidHandler = Handler(Looper.getMainLooper())
    private val pidRunnable = object : Runnable {
        override fun run() { if (targetHeading != null) runPidLoop(PID_INTERVAL_MS / 1000.0); pidHandler.postDelayed(this, PID_INTERVAL_MS) }
    }
    private val workerHandler = Handler(HandlerThread("IMU").apply { start() }.looper)
    private val sharedBuffer = ByteBuffer
        .allocate(32)
        .order(ByteOrder.LITTLE_ENDIAN)

    private var isKeepScreenOn = false; private var isLogging = false
    private var gpsHeadingOffset = 0.0; private var currentGpsSpeedKnots = 0.0; private var minGpsSpeedKnots = 2.0
    private var logFile: File? = null
    private var logStream: FileOutputStream? = null
    private var lastTimestamp = 0L; private var lastUiUpdateTime = 0L

    companion object {
        val SERVICE_AE30 = UUID.fromString("0000ae30-0000-1000-8000-00805f9b34fb")
        val SERVICE_AE00 = UUID.fromString("0000ae00-0000-1000-8000-00805f9b34fb")
        val CHAR_DATA = UUID.fromString("0000ae02-0000-1000-8000-00805f9b34fb")
        val CHAR_MOTOR = UUID.fromString("0000ae03-0000-1000-8000-00805f9b34fb")

        var imuGattInstance: BluetoothGatt? = null
        var gpsGattInstance: BluetoothGatt? = null
        var activeServiceUuid: UUID? = null
        var latestHeading = 0.0
        var latestSpeed = 0.0
        var latestRoll = 0.0
        var latestPitch = 0.0
        var latestRawMagHeading = 0.0
        var motorMode = 0 // 0: ESC, 1: BLDC

        @SuppressLint("MissingPermission")
        fun sendMotorCommand(gatt: BluetoothGatt?, cmd: Int, port: Int, stbd: Int) {
            // Priority order: explicit gatt > external GPS device > IMU device
            val deviceGatt = gatt ?: gpsGattInstance ?: imuGattInstance ?: return
            
            val service = deviceGatt.getService(SERVICE_AE30) ?: deviceGatt.getService(SERVICE_AE00) ?: return
            val characteristic = service.getCharacteristic(CHAR_MOTOR) ?: return
            
            val data = ByteArray(5)
            data[0] = cmd.toByte()
            data[1] = (port and 0xFF).toByte()
            data[2] = ((port shr 8) and 0xFF).toByte()
            data[3] = (stbd and 0xFF).toByte()
            data[4] = ((stbd shr 8) and 0xFF).toByte()
            
            characteristic.value = data
            characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
            deviceGatt.writeCharacteristic(characteristic)
        }
    }

    private val scanLauncher = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { r -> 
        if (r.resultCode == Activity.RESULT_OK) {
            val address = r.data?.getStringExtra("device_address")
            val name = r.data?.getStringExtra("device_name")
            if (address != null) connectToDevice(address, name)
        }
    }
    private val requestPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { result ->

            val denied = result.filterValues { !it }.keys

            if (denied.isEmpty()) {
                openScanActivity()
                startGpsUpdates()
            } else {
                Toast.makeText(
                    this,
                    "Permissions denied: $denied",
                    Toast.LENGTH_LONG
                ).show()
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        prefs = getSharedPreferences("calib_prefs", Context.MODE_PRIVATE)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        loadCalibration()

        binding.btnScan.setOnClickListener { checkPermissionsAndScan() }
        binding.btnCalibrateMag.setOnClickListener { startMagCalibration() }
        binding.btnCalibrateGyro.setOnClickListener { startGyroCalibration() }
        binding.btnResetGps.setOnClickListener { gpsHeadingOffset = 0.0; saveCalibration(); updateUi(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) }
        
        binding.btnKeepScreenOn.setOnClickListener { toggleKeepScreenOn() }
        binding.btnLogToggle.setOnClickListener { toggleLogging() }
        binding.btnToggleDisplay.setOnClickListener { toggleDisplayMode() }
        
        binding.btnMotorPage.setOnClickListener {
            startActivity(Intent(this, MotorControlActivity::class.java))
        }

        binding.btnPidHold.setOnClickListener { setTarget((currentHeading + gpsHeadingOffset + 360) % 360) }
        binding.btnPidM10.setOnClickListener { adjustTarget(-10.0) }
        binding.btnPidM1.setOnClickListener { adjustTarget(-1.0) }
        binding.btnPidP1.setOnClickListener { adjustTarget(1.0) }
        binding.btnPidP10.setOnClickListener { adjustTarget(10.0) }
        binding.btnPidStop.setOnClickListener { 
            targetHeading = null
            stopLogging()
            updatePidUi()
            stopMotors() 
        }

        binding.btnThrM5.setOnClickListener { adjustThrottle(-5.0) }
        binding.btnThrM2.setOnClickListener { adjustThrottle(-2.0) }
        binding.btnThrP2.setOnClickListener { adjustThrottle(2.0) }
        binding.btnThrP5.setOnClickListener { adjustThrottle(5.0) }
        
        setupSpinners()
        setupPidInputs()
        pidHandler.post(pidRunnable)
    }

    override fun onStart() {
        super.onStart()
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            startGpsUpdates()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopLogging()
    }

    private fun toggleDisplayMode() {
        displayMode = if (displayMode == 0) 1 else 0
        binding.btnToggleDisplay.text = if (displayMode == 0) "IMU" else "GPS"
        resetDisplay()
    }

    private fun resetDisplay() {
        runOnUiThread {
            binding.txtHeading.text = "--"
            binding.txtRollPitch.text = "R: 0.0° P: 0.0° M: 0°"
            binding.txtRollPitch.setTextColor(0xFF616161.toInt())
            binding.txtGpsInfo.text = "0.0 kn"
            binding.txtGpsOffset.text = "0.0°"
            binding.txtSeaState.text = "Sea: 0"
            if (!isConnected && !isGpsConnected) {
                binding.txtStatus.text = "Status: Disconnected"
            }
        }
    }

    private fun toggleKeepScreenOn() {
        isKeepScreenOn = !isKeepScreenOn
        window.apply { if (isKeepScreenOn) addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) else clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) }
        binding.btnKeepScreenOn.text = "Screen: ${if (isKeepScreenOn) "ON" else "OFF"}"
        binding.btnKeepScreenOn.backgroundTintList = ColorStateList.valueOf(if (isKeepScreenOn) 0xFF4CAF50.toInt() else 0xFF607D8B.toInt())
    }

    private fun toggleLogging() {
        if (isLogging) stopLogging() else startLogging()
    }

    private fun startLogging() {
        if (isLogging) return
        try {
            logFile = File(getExternalFilesDir(null), "nav_log_${System.currentTimeMillis()}.txt")
            logStream = FileOutputStream(logFile, true)
            isLogging = true
            runOnUiThread {
                binding.btnLogToggle.text = "Log: ON"
                binding.btnLogToggle.backgroundTintList = ColorStateList.valueOf(0xFF4CAF50.toInt())
            }
            writeLog("INFO", "Logging Started")
        } catch (e: Exception) {
            Log.e("Logging", "Failed to start", e)
        }
    }

    private fun stopLogging() {
        if (!isLogging) return
        isLogging = false
        val stream = logStream
        logStream = null
        workerHandler.post {
            try {
                stream?.write("${System.currentTimeMillis()},INFO,Logging Stopped\n".toByteArray())
                stream?.flush()
                stream?.close()
            } catch (e: Exception) { }
        }
        runOnUiThread {
            binding.btnLogToggle.text = "Log: OFF"
            binding.btnLogToggle.backgroundTintList = ColorStateList.valueOf(0xFF607D8B.toInt())
        }
    }

    private fun writeLog(tag: String, msg: String) {
        if (!isLogging) return
        workerHandler.post {
            try {
                logStream?.write("${System.currentTimeMillis()},$tag,$msg\n".toByteArray())
            } catch (e: Exception) { }
        }
    }

    private fun loadCalibration() {
        minMx = prefs.getFloat("minMx", 1e6f).toDouble()
        maxMx = prefs.getFloat("maxMx", -1e6f).toDouble()
        minMy = prefs.getFloat("minMy", 1e6f).toDouble()
        maxMy = prefs.getFloat("maxMy", -1e6f).toDouble()
        minMz = prefs.getFloat("minMz", 1e6f).toDouble()
        maxMz = prefs.getFloat("maxMz", -1e6f).toDouble()
        offsetX = (minMx + maxMx) / 2.0
        offsetY = (minMy + maxMy) / 2.0
        offsetZ = (minMz + maxMz) / 2.0
        val avgDeltaX = (maxMx - minMx) / 2.0
        val avgDeltaY = (maxMy - minMy) / 2.0
        val avgDeltaZ = (maxMz - minMz) / 2.0
        val avgDelta = (avgDeltaX + avgDeltaY + avgDeltaZ) / 3.0
        scaleX = if (avgDeltaX > 0) avgDelta / avgDeltaX else 1.0
        scaleY = if (avgDeltaY > 0) avgDelta / avgDeltaY else 1.0
        scaleZ = if (avgDeltaZ > 0) avgDelta / avgDeltaZ else 1.0

        gyroOffX = prefs.getFloat("gyroOffX", 0f).toDouble()
        gyroOffY = prefs.getFloat("gyroOffY", 0f).toDouble()
        gyroOffZ = prefs.getFloat("gyroOffZ", 0f).toDouble()
        
        gpsHeadingOffset = prefs.getFloat("gpsHeadingOffset", 0f).toDouble()
        minGpsSpeedKnots = prefs.getFloat("minGpsSpeedKnots", 2.0f).toDouble()
        deadband = prefs.getFloat("deadband", 3.0f).toDouble()
        isAutoDeadband = prefs.getBoolean("isAutoDeadband", false)
        
        escThrottle = prefs.getFloat("escThrottle", 580f).toDouble()
        bldcThrottle = prefs.getFloat("bldcThrottle", 2000f).toDouble()
        motorMode = prefs.getInt("motorMode", 0)

        pidKp = prefs.getFloat("pidKp", 0.8f).toDouble()
        pidKi = prefs.getFloat("pidKi", 0.02f).toDouble()
        pidKd = prefs.getFloat("pidKd", 0.5f).toDouble()
    }

    private fun saveCalibration() {
        prefs.edit().apply {
            putFloat("minMx", minMx.toFloat())
            putFloat("maxMx", maxMx.toFloat())
            putFloat("minMy", minMy.toFloat())
            putFloat("maxMy", maxMy.toFloat())
            putFloat("minMz", minMz.toFloat())
            putFloat("maxMz", maxMz.toFloat())
            putFloat("gyroOffX", gyroOffX.toFloat())
            putFloat("gyroOffY", gyroOffY.toFloat())
            putFloat("gyroOffZ", gyroOffZ.toFloat())
            putFloat("gpsHeadingOffset", gpsHeadingOffset.toFloat())
            putFloat("minGpsSpeedKnots", minGpsSpeedKnots.toFloat())
            putFloat("deadband", deadband.toFloat())
            putBoolean("isAutoDeadband", isAutoDeadband)
            putFloat("escThrottle", escThrottle.toFloat())
            putFloat("bldcThrottle", bldcThrottle.toFloat())
            putInt("motorMode", motorMode)
            putFloat("pidKp", pidKp.toFloat())
            putFloat("pidKi", pidKi.toFloat())
            putFloat("pidKd", pidKd.toFloat())
            apply()
        }
    }

    private fun setupSpinners() {
        val speeds = arrayOf("2.0", "2.5", "3.0", "3.5", "4.0")
        binding.spinnerMinSpeed.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_item, speeds).apply { setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item) }
        binding.spinnerMinSpeed.setSelection(speeds.indexOf(String.format(Locale.US, "%.1f", minGpsSpeedKnots)).coerceAtLeast(0))
        binding.spinnerMinSpeed.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) { minGpsSpeedKnots = speeds[pos].toDouble(); saveCalibration() }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }

        val dbs = (3..15).map { it.toString() }.toTypedArray()
        binding.spinnerDeadband.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_item, dbs).apply { setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item) }
        binding.spinnerDeadband.setSelection(dbs.indexOf(deadband.toInt().toString()).coerceAtLeast(0))
        binding.spinnerDeadband.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) { deadband = dbs[pos].toDouble(); saveCalibration() }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }
        binding.cbAutoDeadband.isChecked = isAutoDeadband
        binding.cbAutoDeadband.setOnCheckedChangeListener { _, isChecked -> isAutoDeadband = isChecked; saveCalibration() }
    }

    private fun setupPidInputs() {
        binding.editPidP.setText(String.format(Locale.US, "%.2f", pidKp))
        binding.editPidI.setText(String.format(Locale.US, "%.3f", pidKi))
        binding.editPidD.setText(String.format(Locale.US, "%.2f", pidKd))
        val watcher = object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, st: Int, c: Int, a: Int) {}
            override fun onTextChanged(s: CharSequence?, st: Int, b: Int, c: Int) {}
            override fun afterTextChanged(s: Editable?) { try { pidKp = binding.editPidP.text.toString().toDoubleOrNull() ?: pidKp; pidKi = binding.editPidI.text.toString().toDoubleOrNull() ?: pidKi; pidKd = binding.editPidD.text.toString().toDoubleOrNull() ?: pidKd; saveCalibration() } catch (e: Exception) {} }
        }
        binding.editPidP.addTextChangedListener(watcher)
        binding.editPidI.addTextChangedListener(watcher)
        binding.editPidD.addTextChangedListener(watcher)
        
        updateThrottleUi()
    }

    private fun updateThrottleUi() {
        val thr = if (motorMode == 0) escThrottle else bldcThrottle
        binding.txtThrottle.text = String.format(Locale.US, "Thr: %.0f", thr)
        binding.txtThrottleBig.text = String.format(Locale.US, "T: %.0f", thr)
    }

    private fun adjustThrottle(delta: Double) {
        if (motorMode == 0) {
            escThrottle = (escThrottle + delta).coerceIn(500.0, 1000.0)
        } else {
            bldcThrottle = (bldcThrottle + delta * 10).coerceIn(1000.0, 3000.0)
        }
        updateThrottleUi()
        saveCalibration()
    }

    private fun adjustTarget(delta: Double) {
        targetHeading = targetHeading?.let { (it + delta + 360) % 360 }
        updatePidUi()
    }

    private fun setTarget(h: Double) {
        targetHeading = h
        pidIntegral = 0.0
        pidLastError = 0.0
        updatePidUi()
        startLogging()
    }

    private fun updatePidUi() {
        runOnUiThread {
            binding.txtTarget.text = targetHeading?.let { String.format(Locale.US, "Target: %.0f°", it) } ?: "Target: OFF"
            binding.txtTarget.setTextColor(if (targetHeading != null) 0xFF4CAF50.toInt() else 0xFF616161.toInt())
        }
    }

    private fun stopMotors() {
        sendMotorCommand(null, 0, 0, 0)
    }

    private fun runPidLoop(dt: Double) {
        val target = targetHeading ?: return
        val current = (currentHeading + gpsHeadingOffset + 360) % 360
        
        var error = target - current
        if (error > 180) error -= 360
        if (error < -180) error += 360
        
        val absError = abs(error)
        val currentDeadband = if (isAutoDeadband) (currentSeaState * 1.5 + 1.0) else deadband
        
        if (absError < currentDeadband) {
            stopMotors()
            pidIntegral = 0.0
            return
        }

        pidIntegral += error * dt
        pidIntegral = pidIntegral.coerceIn(-50.0, 50.0)
        val derivative = (error - pidLastError) / dt
        pidLastError = error
        
        val output = pidKp * error + pidKi * pidIntegral + pidKd * derivative
        
        val baseThr = if (motorMode == 0) escThrottle else bldcThrottle
        val diff = output.toInt().coerceIn(-400, 400)
        
        val port = (baseThr + diff).toInt()
        val stbd = (baseThr - diff).toInt()
        
        sendMotorCommand(null, 1, port, stbd)
        writeLog("PID", "T:$target C:$current E:$error O:$output P:$port S:$stbd Sea:$currentSeaState DB:$currentDeadband")
    }

    private fun checkPermissionsAndScan() {
        val perms = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            perms.add(Manifest.permission.BLUETOOTH_SCAN)
            perms.add(Manifest.permission.BLUETOOTH_CONNECT)
        }
        requestPermissionLauncher.launch(perms.toTypedArray())
    }

    private fun openScanActivity() {
        scanLauncher.launch(Intent(this, ScanActivity::class.java))
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(address: String, name: String?) {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val device = bluetoothManager.adapter.getRemoteDevice(address)
        val isGpsDevice = name?.contains("GPS", ignoreCase = true) == true
        
        val callback = object : BluetoothGattCallback() {
            override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    gatt.discoverServices()
                    runOnUiThread { 
                        if (isGpsDevice) isGpsConnected = true else isConnected = true
                        updateStatus()
                    }
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    gatt.close()
                    runOnUiThread { 
                        if (isGpsDevice) {
                            isGpsConnected = false
                            gpsGatt = null
                            gpsGattInstance = null
                        } else {
                            isConnected = false
                            imuGatt = null
                            imuGattInstance = null
                        }
                        updateStatus()
                    }
                }
            }

            override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
                val service = gatt.getService(SERVICE_AE30) ?: gatt.getService(SERVICE_AE00)
                if (service != null) {
                    activeServiceUuid = service.uuid
                    val char = service.getCharacteristic(CHAR_DATA)
                    if (char != null) {
                        gatt.setCharacteristicNotification(char, true)
                        val desc = char.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
                        desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                        gatt.writeDescriptor(desc)
                    }
                }
            }

            override fun onCharacteristicChanged(gatt: BluetoothGatt, char: BluetoothGattCharacteristic) {
                if (char.uuid == CHAR_DATA) {
                    if (isGpsDevice) processGpsData(char.value) else processImuData(char.value)
                }
            }
        }

        if (isGpsDevice) {
            gpsGatt = device.connectGatt(this, false, callback)
            gpsGattInstance = gpsGatt
        } else {
            imuGatt = device.connectGatt(this, false, callback)
            imuGattInstance = imuGatt
        }
    }

    private fun updateStatus() {
        val status = when {
            isConnected && isGpsConnected -> "Connected: IMU + GPS"
            isConnected -> "Connected: IMU"
            isGpsConnected -> "Connected: GPS"
            else -> "Status: Disconnected"
        }
        binding.txtStatus.text = status
        binding.txtStatus.setTextColor(if (isConnected || isGpsConnected) 0xFF4CAF50.toInt() else 0xFFF44336.toInt())
    }

    private fun processImuData(data: ByteArray) {
        if (data.size < 24) return
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        
        var ax = buffer.getFloat(); var ay = buffer.getFloat(); var az = buffer.getFloat()
        var mx = buffer.getFloat(); var my = buffer.getFloat(); var mz = buffer.getFloat()
        
        if (ACCT_ROTATED_180) { ax = -ax; ay = -ay; mx = -mx; my = -my }
        if (INVERT_MAG_Z) { mz = -mz }

        if (isCalibratingMag) {
            minMx = min(minMx, mx.toDouble()); maxMx = max(maxMx, mx.toDouble())
            minMy = min(minMy, my.toDouble()); maxMy = max(maxMy, my.toDouble())
            minMz = min(minMz, mz.toDouble()); maxMz = max(maxMz, mz.toDouble())
            return
        }

        val cx = (mx - offsetX) * scaleX
        val cy = (my - offsetY) * scaleY
        val cz = (mz - offsetZ) * scaleZ

        val roll = atan2(ay.toDouble(), az.toDouble())
        val pitch = atan2(-ax.toDouble(), sqrt(ay * ay + az * az).toDouble())

        val cosR = cos(roll); val sinR = sin(roll)
        val cosP = cos(pitch); val sinP = sin(pitch)

        val xh = cx * cosP + cy * sinP * sinR + cz * sinP * cosR
        val yh = cy * cosR - cz * sinR
        
        var heading = Math.toDegrees(atan2(-yh, xh))
        if (heading < 0) heading += 360.0
        
        latestRoll = Math.toDegrees(roll)
        latestPitch = Math.toDegrees(pitch)
        latestRawMagHeading = heading

        if (data.size >= 36) {
            var gx = Math.toRadians(buffer.getFloat(24).toDouble() - gyroOffX)
            var gy = Math.toRadians(buffer.getFloat(28).toDouble() - gyroOffY)
            var gz = Math.toRadians(buffer.getFloat(32).toDouble() - gyroOffZ)
            
            if (ACCT_ROTATED_180) { gx = -gx; gy = -gy }
            if (INVERT_GYRO_Z) { gz = -gz }

            if (isCalibratingGyro) {
                gyroSumX += buffer.getFloat(24); gyroSumY += buffer.getFloat(28); gyroSumZ += buffer.getFloat(32)
                gyroCount++
                return
            }

            val now = System.currentTimeMillis()
            if (lastMagUpdateTime != 0L) {
                val dt = (now - lastMagUpdateTime) / 1000.0
                val gyroRate = gz
                kfHeading += gyroRate * dt
                kfP += kfQ
            }
            lastMagUpdateTime = now
        }

        val magHeadingRad = Math.toRadians(heading)
        var diff = magHeadingRad - kfHeading
        while (diff > PI) diff -= 2 * PI
        while (diff < -PI) diff += 2 * PI
        
        val r = if (abs(latestRoll) > 15 || abs(latestPitch) > 15) kfR_mag_low_weight else kfR_mag_normal
        val k = kfP / (kfP + r)
        kfHeading += k * diff
        kfP *= (1 - k)
        
        currentHeading = (Math.toDegrees(kfHeading) + 360) % 360
        latestHeading = currentHeading

        updateSeaState(latestRoll, latestPitch)
        
        val ts = System.currentTimeMillis()
        if (ts - lastUiUpdateTime > 100) {
            lastUiUpdateTime = ts
            updateUi(currentHeading, latestRoll, latestPitch, mx.toDouble(), my.toDouble(), mz.toDouble(), xh, yh, heading)
        }
    }

    private fun processGpsData(data: ByteArray) {
        if (data.size < 8) return
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        val gpsHeading = buffer.getFloat()
        val gpsSpeed = buffer.getFloat()
        
        latestSpeed = gpsSpeed.toDouble()
        currentGpsSpeedKnots = gpsSpeed.toDouble()

        if (gpsSpeed > minGpsSpeedKnots) {
            val gpsHeadingRad = Math.toRadians(gpsHeading.toDouble())
            var diff = gpsHeadingRad - kfHeading
            while (diff > PI) diff -= 2 * PI
            while (diff < -PI) diff += 2 * PI
            
            val k = kfP / (kfP + kfR_gps)
            kfHeading += k * diff
            kfP *= (1 - k)
            
            val magHeading = currentHeading
            var offsetErr = gpsHeading - magHeading
            while (offsetErr > 180) offsetErr -= 360
            while (offsetErr < -180) offsetErr += 360
            
            gpsHeadingOffset += offsetErr * 0.01 
            while (gpsHeadingOffset > 180) gpsHeadingOffset -= 360
            while (gpsHeadingOffset < -180) gpsHeadingOffset += 360
        }

        runOnUiThread {
            binding.txtGpsInfo.text = String.format(Locale.US, "%.1f kn", gpsSpeed)
            binding.txtGpsOffset.text = String.format(Locale.US, "%.1f°", gpsHeadingOffset)
        }
    }

    private fun updateSeaState(roll: Double, pitch: Double) {
        val motion = abs(roll) + abs(pitch)
        motionBuffer.add(motion)
        if (motionBuffer.size > BUFFER_SIZE) motionBuffer.removeAt(0)
        
        val avgMotion = motionBuffer.average()
        currentSeaState = when {
            avgMotion < 3.0 -> 1
            avgMotion < 8.0 -> 2
            avgMotion < 15.0 -> 3
            else -> 4
        }
        runOnUiThread { binding.txtSeaState.text = "Sea: $currentSeaState" }
    }

    private fun updateUi(h: Double, r: Double, p: Double, mx: Double, my: Double, mz: Double, xh: Double, yh: Double, rawMag: Double) {
        runOnUiThread {
            val correctedHeading = (h + gpsHeadingOffset + 360) % 360
            
            if (displayMode == 0) {
                binding.txtHeading.text = String.format(Locale.US, "%03.0f", correctedHeading)
                binding.txtRollPitch.text = String.format(Locale.US, "R:%.1f P:%.1f M:%03.0f", r, p, rawMag)
                binding.txtRollPitch.setTextColor(if (abs(r) > 20 || abs(p) > 20) Color.RED else 0xFF616161.toInt())
            } else {
                binding.txtHeading.text = String.format(Locale.US, "%03.1f", correctedHeading)
                binding.txtRollPitch.text = String.format(Locale.US, "Raw Mag: %.1f°", rawMag)
            }
            
            updateThrottleUi()

            if (isLogging) {
                writeLog("DATA", String.format(Locale.US, "H:%.2f,R:%.2f,P:%.2f,MX:%.1f,MY:%.1f,MZ:%.1f,S:%.1f", h, r, p, mx, my, mz, currentGpsSpeedKnots))
            }
        }
    }

    private fun startMagCalibration() {
        isCalibratingMag = true
        minMx = 1e6; maxMx = -1e6; minMy = 1e6; maxMy = -1e6; minMz = 1e6; maxMz = -1e6
        calDialog = AlertDialog.Builder(this)
            .setTitle("Mag Calibration")
            .setMessage("Rotate device in all directions...")
            .setPositiveButton("Done") { _, _ -> 
                isCalibratingMag = false
                loadCalibration()
                saveCalibration()
            }
            .show()
    }

    private fun startGyroCalibration() {
        isCalibratingGyro = true
        gyroSumX = 0.0; gyroSumY = 0.0; gyroSumZ = 0.0
        gyroCount = 0
        
        calDialog = AlertDialog.Builder(this)
            .setTitle("Gyro Calibration")
            .setMessage("Keep device perfectly still...")
            .setCancelable(false)
            .create()
        calDialog?.show()

        gyroCalibTimer = object : CountDownTimer(5000, 1000) {
            override fun onTick(ms: Long) { calDialog?.setMessage("Keep still... ${ms/1000}s") }
            override fun onFinish() {
                isCalibratingGyro = false
                if (gyroCount > 0) {
                    gyroOffX = gyroSumX / gyroCount
                    gyroOffY = gyroSumY / gyroCount
                    gyroOffZ = gyroSumZ / gyroCount
                    saveCalibration()
                }
                calDialog?.dismiss()
                Toast.makeText(this@MainActivity, "Gyro Calibrated", Toast.LENGTH_SHORT).show()
            }
        }.start()
    }

    @SuppressLint("MissingPermission")
    private fun startGpsUpdates() {
        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000, 0f, object : LocationListener {
                override fun onLocationChanged(location: Location) {
                    val declination = GeomagneticField(
                        location.latitude.toFloat(),
                        location.longitude.toFloat(),
                        location.altitude.toFloat(),
                        System.currentTimeMillis()
                    ).declination
                    magneticDeclination = declination
                }
                override fun onStatusChanged(p: String?, s: Int, e: Bundle?) {}
                override fun onProviderEnabled(p: String) {}
                override fun onProviderDisabled(p: String) {}
            })
        } catch (e: Exception) { }
    }
}
