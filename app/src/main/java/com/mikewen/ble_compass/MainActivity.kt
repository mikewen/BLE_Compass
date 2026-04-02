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
            binding.btnLogToggle.backgroundTintList = ColorStateList.valueOf(0xFF9E9E9E.toInt())
        }
    }

    private fun writeLog(tag: String, data: String) {
        val stream = logStream ?: return
        val line = "${System.currentTimeMillis()},$tag,$data\n"
        workerHandler.post {
            try {
                stream.write(line.toByteArray())
            } catch (e: Exception) {
                Log.e("Logging", "Write failed", e)
            }
        }
    }

    private fun setTarget(h: Double) { 
        if (targetHeading == null) startLogging()
        targetHeading = h
        pidIntegral = 0.0
        updatePidUi() 
    }
    private fun adjustTarget(d: Double) { targetHeading?.let { setTarget((it + d + 360) % 360) } }

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
        binding.editPidP.addTextChangedListener(watcher); binding.editPidI.addTextChangedListener(watcher); binding.editPidD.addTextChangedListener(watcher)
    }

    private fun loadCalibration() {
        offsetX = prefs.getFloat("mag_off_x", 0.0f).toDouble(); offsetY = prefs.getFloat("mag_off_y", 0.0f).toDouble(); offsetZ = prefs.getFloat("mag_off_z", 0.0f).toDouble()
        scaleX = prefs.getFloat("mag_scale_x", 1.0f).toDouble(); scaleY = prefs.getFloat("mag_scale_y", 1.0f).toDouble(); scaleZ = prefs.getFloat("mag_scale_z", 1.0f).toDouble()
        gyroOffX = prefs.getFloat("gyro_off_x", 0.0f).toDouble(); gyroOffY = prefs.getFloat("gyro_off_y", 0.0f).toDouble(); gyroOffZ = prefs.getFloat("gyro_off_z", 0.0f).toDouble()
        gpsHeadingOffset = prefs.getFloat("gps_heading_offset", 0.0f).toDouble()
        minGpsSpeedKnots = prefs.getFloat("min_gps_speed", 2.0f).toDouble(); deadband = prefs.getFloat("pid_deadband", 3.0f).toDouble()
        isAutoDeadband = prefs.getBoolean("auto_deadband", false)
        pidKp = prefs.getFloat("pid_kp", 0.8f).toDouble(); pidKi = prefs.getFloat("pid_ki", 0.02f).toDouble(); pidKd = prefs.getFloat("pid_kd", 0.5f).toDouble()
        magneticDeclination = prefs.getFloat("mag_declination", 0.0f)
    }

    private fun saveCalibration() {
        prefs.edit().apply {
            putFloat("mag_off_x", offsetX.toFloat()); putFloat("mag_off_y", offsetY.toFloat()); putFloat("mag_off_z", offsetZ.toFloat())
            putFloat("mag_scale_x", scaleX.toFloat()); putFloat("mag_scale_y", scaleY.toFloat()); putFloat("mag_scale_z", scaleZ.toFloat())
            putFloat("gyro_off_x", gyroOffX.toFloat()); putFloat("gyro_off_y", gyroOffY.toFloat()); putFloat("gyro_off_z", gyroOffZ.toFloat())
            putFloat("gps_heading_offset", gpsHeadingOffset.toFloat())
            putFloat("min_gps_speed", minGpsSpeedKnots.toFloat()); putFloat("pid_deadband", deadband.toFloat())
            putBoolean("auto_deadband", isAutoDeadband)
            putFloat("pid_kp", pidKp.toFloat()); putFloat("pid_ki", pidKi.toFloat()); putFloat("pid_kd", pidKd.toFloat())
            putFloat("mag_declination", magneticDeclination)
            apply()
        }
    }

    private fun openScanActivity() {
        scanLauncher.launch(Intent(this, ScanActivity::class.java))
    }

    private fun checkPermissionsAndScan() {
        val permissions = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        }
        
        val missing = permissions.filter { ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isEmpty()) {
            openScanActivity()
        } else {
            requestPermissionLauncher.launch(missing.toTypedArray())
        }
    }

    private fun startGpsUpdates() {
        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 1f, phoneLocationListener)
        } catch (e: SecurityException) {
            Log.e("GPS", "Permission denied for phone GPS")
        }
    }

    private fun updateDeclination(l: Location) {
        val now = System.currentTimeMillis()
        if (now - lastDeclinationUpdate > 600000) { // Update every 10 mins
            val field = GeomagneticField(l.latitude.toFloat(), l.longitude.toFloat(), l.altitude.toFloat(), now)
            if (field.declination != magneticDeclination) {
                magneticDeclination = field.declination
                saveCalibration()
            }
            lastDeclinationUpdate = now
            if (isLogging) writeLog("DECL", "%.4f at %.4f,%.4f".format(magneticDeclination, l.latitude, l.longitude))
        }
    }

    private val phoneLocationListener = object : LocationListener {
        override fun onLocationChanged(l: Location) {
            val now = System.currentTimeMillis()
            updateDeclination(l)

            // Phone GPS is automatically ignored if the external BLE GPS is connected and 0xA3 is available and valid (within 5 seconds)
            if (isGpsConnected && (now - lastA3Timestamp < 5000)) return
            
            currentGpsSpeedKnots = l.speed * 1.94384
            latestSpeed = currentGpsSpeedKnots
            
            if (isLogging) {
                writeLog("PHONE_GPS", "%.3f,%.2f".format(l.speed, l.bearing))
            }

            // Only update GPS if boatSpeed > 0.8 m/s
            if (l.speed > 0.8f && l.hasBearing() && now - lastGpsKalmanUpdateTime >= 1000) {
                lastGpsKalmanUpdateTime = now
                runKalmanUpdate(Math.toRadians(l.bearing.toDouble()), kfR_gps)
                currentHeading = (Math.toDegrees(kfHeading) + 360.0) % 360.0
                latestHeading = currentHeading
            }

            if (!isConnected && l.hasBearing() && displayMode == 1) {
                currentHeading = l.bearing.toDouble()
                latestHeading = currentHeading
                updateUi(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            } else if (isConnected && currentSeaState <= 2 && currentGpsSpeedKnots >= minGpsSpeedKnots && l.hasBearing()) {
                val magH = (currentHeading + 360) % 360
                var diff = l.bearing.toDouble() - magH
                while (diff > 180) diff -= 360; while (diff < -180) diff += 360
                gpsHeadingOffset = 0.995 * gpsHeadingOffset + 0.005 * diff
                saveCalibration()
            }
        }
        override fun onStatusChanged(p: String?, s: Int, e: Bundle?) {}
        override fun onProviderEnabled(p: String) {}
        override fun onProviderDisabled(p: String) {}
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(address: String, name: String?) {
        val dev = (getSystemService(BLUETOOTH_SERVICE) as BluetoothManager).adapter.getRemoteDevice(address)
        val callback = object : BluetoothGattCallback() {
            override fun onConnectionStateChange(g: BluetoothGatt, s: Int, newState: Int) {
                runOnUiThread {
                    if (newState == BluetoothProfile.STATE_CONNECTED) {
                        if (name == "GPS_PWM") {
                            isGpsConnected = true
                            gpsGattInstance = g
                        } else {
                            isConnected = true
                            imuGattInstance = g
                        }
                        g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                        g.discoverServices()
                    } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                        if (name == "GPS_PWM") { 
                            isGpsConnected = false
                            gpsGatt = null
                            gpsGattInstance = null
                        } 
                        else { 
                            isConnected = false
                            imuGatt = null
                            imuGattInstance = null 
                        }
                        
                        if (!isConnected && !isGpsConnected) {
                            resetDisplay()
                            binding.btnScan.text = "Connect"
                            binding.txtStatus.text = "Status: Disconnected"
                        } else {
                            updateStatusText()
                        }
                    }
                    binding.btnToggleDisplay.text = if (displayMode == 0) "IMU" else "GPS"
                }
            }
            override fun onServicesDiscovered(g: BluetoothGatt, s: Int) {
                val service = g.getService(SERVICE_AE30) ?: g.getService(SERVICE_AE00)
                if (service != null && g.device.name?.startsWith("IMU_") == true) {
                    activeServiceUuid = service.uuid
                }

                service?.getCharacteristic(CHAR_DATA)?.let {
                    g.setCharacteristicNotification(it, true)
                    it.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))?.let { d -> 
                        d.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                        g.writeDescriptor(d)
                    }
                }
                runOnUiThread { updateStatusText() }
            }
            override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
                val dataCopy = c.value.clone()
                workerHandler.post {
                    processRawData(dataCopy)
                }
            }
        }
        
        if (name == "GPS_PWM") {
            gpsGatt = dev.connectGatt(this, false, callback)
        } else {
            imuGatt = dev.connectGatt(this, false, callback)
        }
    }

    private fun updateStatusText() {
        val statusText = mutableListOf<String>()
        if (isConnected) statusText.add("IMU")
        if (isGpsConnected) statusText.add("GPS")
        binding.btnScan.text = if (statusText.isEmpty()) "Connect" else statusText.joinToString("+")
        binding.txtStatus.text = "Status: Connected (" + statusText.joinToString(", ") + ")"
    }

    private inline fun getS16LE(d: ByteArray, i: Int): Int {
        return (d[i].toInt() and 0xFF) or (d[i + 1].toInt() shl 8)
    }

    private inline fun getU16LE(d: ByteArray, i: Int): Int {
        return (d[i].toInt() and 0xFF) or ((d[i + 1].toInt() and 0xFF) shl 8)
    }

    private inline fun getS32LE(d: ByteArray, i: Int): Int {
        return (d[i].toInt() and 0xFF) or
                ((d[i + 1].toInt() and 0xFF) shl 8) or
                ((d[i + 2].toInt() and 0xFF) shl 16) or
                ((d[i + 3].toInt() and 0xFF) shl 24)
    }

    private inline fun getU32LE(d: ByteArray, i: Int): Long {
        return ((d[i].toLong() and 0xFF)) or
                ((d[i + 1].toLong() and 0xFF) shl 8) or
                ((d[i + 2].toLong() and 0xFF) shl 16) or
                ((d[i + 3].toLong() and 0xFF) shl 24)
    }
    private fun processRawData(data: ByteArray) {
        if (data.isEmpty()) return

        val header = data[0].toInt() and 0xFF
        val now = System.currentTimeMillis()

        when (header) {

            // =========================
            // IMU + MAG (A1) - 50Hz
            // =========================
            0xA1 -> {
                if (data.size < 20) return

                val dtRaw = if (lastTimestamp == 0L) 0.02 else (now - lastTimestamp) * 0.001
                val dt = dtRaw.coerceIn(0.01, 0.05)
                lastTimestamp = now

                var ax = getS16LE(data, 2).toDouble()
                var ay = getS16LE(data, 4).toDouble()
                var az = getS16LE(data, 6).toDouble()

                var gx = getS16LE(data, 8).toDouble()
                var gy = getS16LE(data, 10).toDouble()
                var gz = getS16LE(data, 12).toDouble()

                var mx = getS16LE(data, 14).toDouble()
                var my = getS16LE(data, 16).toDouble()
                var mz = getS16LE(data, 18).toDouble()

                if (isCalibratingMag) {
                    if (mx < minMx) minMx = mx; if (mx > maxMx) maxMx = mx
                    if (my < minMy) minMy = my; if (my > maxMy) maxMy = my
                    if (mz < minMz) minMz = mz; if (mz > maxMz) maxMz = mz
                }
                if (isCalibratingGyro) {
                    gyroSumX += gx; gyroSumY += gy; gyroSumZ += gz
                    gyroCount++
                }

                if (ACCT_ROTATED_180) {
                    ax = -ax; ay = -ay
                    gx = -gx; gy = -gy
                }

                mx = (mx - offsetX) * scaleX
                my = (my - offsetY) * scaleY
                mz = (mz - offsetZ) * scaleZ
                if (INVERT_MAG_Z) mz = -mz

                gx = (gx - gyroOffX) / gyroScaleDegS
                gy = (gy - gyroOffY) / gyroScaleDegS
                gz = (gz - gyroOffZ) / gyroScaleDegS
                if (INVERT_GYRO_Z) gz = -gz

                val amag = sqrt(ax * ax + ay * ay + az * az)

                var rollDeg = 0.0
                var pitchDeg = 0.0
                var rawMagHeading = latestRawMagHeading

                if (amag > 1e-3) {
                    val invA = 1.0 / amag
                    val nax = ax * invA
                    val nay = ay * invA
                    val naz = az * invA

                    val phi = atan2(nay, naz)
                    val theta = atan2(-nax, nay * sin(phi) + naz * cos(phi))

                    rollDeg = Math.toDegrees(phi)
                    pitchDeg = Math.toDegrees(theta)

                    val sinPhi = sin(phi)
                    val cosPhi = cos(phi)
                    val sinTheta = sin(theta)
                    val cosTheta = cos(theta)

                    val xh = mx * cosTheta + my * sinPhi * sinTheta + mz * cosPhi * sinTheta
                    val yh = my * cosPhi - mz * sinPhi

                    rawMagHeading = (Math.toDegrees(atan2(yh, xh)) + 360.0) % 360.0

                    latestRoll = rollDeg
                    latestPitch = pitchDeg
                    latestRawMagHeading = rawMagHeading
                }

                // =========================
                // Multi-rate Kalman
                // =========================
                runKalmanPredict(gz, dt) // Gyro at 50Hz
                
                if (now - lastMagUpdateTime >= 100) { // Mag at 10Hz
                    lastMagUpdateTime = now
                    // Apply Magnetic Declination correction to align with True North (GPS reference)
                    val correctedMagHeading = (rawMagHeading + magneticDeclination + 360.0) % 360.0
                    
                    // reduce mag weight when boatSpeed > 2.0 m/s (3.887 knots)
                    val r = if (currentGpsSpeedKnots > 3.887) kfR_mag_low_weight else kfR_mag_normal
                    runKalmanUpdate(Math.toRadians(correctedMagHeading), r)
                }
                
                currentHeading = (Math.toDegrees(kfHeading) + 360.0) % 360.0
                latestHeading = currentHeading

                if (isLogging) {
                    writeLog("A1", "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f".format(
                        ax, ay, az, gx, gy, gz, mx, my, mz, rawMagHeading, currentHeading))
                }

                val motion = abs(amag * (1.0 / 2048.0) - 1.0) + abs(gz * 0.01)
                motionBuffer.add(motion)
                if (motionBuffer.size > BUFFER_SIZE) motionBuffer.removeAt(0)

                val varMotion = if (motionBuffer.size < 5) 0.0 else {
                    val avg = motionBuffer.average()
                    motionBuffer.sumOf { (it - avg) * (it - avg) } / motionBuffer.size
                }

                currentSeaState = when {
                    varMotion < 0.0001 -> 1
                    varMotion < 0.001 -> 2
                    varMotion < 0.005 -> 3
                    varMotion < 0.01 -> 4
                    varMotion < 0.05 -> 5
                    varMotion < 0.1 -> 6
                    varMotion < 0.2 -> 7
                    varMotion < 0.5 -> 8
                    else -> 9
                }

                if (displayMode == 0 && now - lastUiUpdateTime > 50) {
                    lastUiUpdateTime = now
                    updateUi(ax, ay, az, mx, my, mz, rollDeg, pitchDeg, rawMagHeading)
                }
            }

            // =========================
            // GPS ORI (A2) - 1Hz
            // =========================
            0xA2 -> {
                if (data.size < 17) return
                val quality = data[5].toInt() and 0xFF
                val pitch = getS16LE(data, 8) * 0.01
                val roll = getS16LE(data, 10) * 0.01
                val heading = getU16LE(data, 12) * 0.01

                if (isLogging) {
                    writeLog("A2", "$quality,%.2f,%.2f,%.2f".format(pitch, roll, heading))
                }

                // Use A2 heading to update Kalman if quality=4, not speed dependent
                if (quality == 4 && now - lastGpsKalmanUpdateTime >= 1000) {
                    lastGpsKalmanUpdateTime = now
                    runKalmanUpdate(Math.toRadians(heading), kfR_gps)
                    currentHeading = (Math.toDegrees(kfHeading) + 360.0) % 360.0
                    latestHeading = currentHeading
                }

                if (displayMode == 1) {
                    latestRoll = roll
                    latestPitch = pitch
                    updateUi(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, roll, pitch, 0.0)
                }
            }

            // =========================
            // GPS POS (A3) - 1Hz
            // =========================
            0xA3 -> {
                if (data.size < 21) return
                val latRaw = getS32LE(data, 5)
                val lonRaw = getS32LE(data, 9)
                val speedKnots = getU16LE(data, 13) * 0.01
                val course = getU16LE(data, 15) * 0.01
                
                // Update declination if we have valid coordinates
                if (latRaw != 0 && lonRaw != 0) {
                    val l = Location("BLE").apply {
                        latitude = latRaw * 1e-7
                        longitude = lonRaw * 1e-7
                    }
                    updateDeclination(l)
                }

                lastA3Timestamp = now
                currentGpsSpeedKnots = speedKnots
                latestSpeed = speedKnots

                if (isLogging) {
                    writeLog("A3", "%.2f,%.2f".format(speedKnots, course))
                }

                // Only update GPS if boatSpeed > 0.8 m/s (1.555 knots)
                if (speedKnots > 1.555 && now - lastGpsKalmanUpdateTime >= 1000) {
                    lastGpsKalmanUpdateTime = now
                    runKalmanUpdate(Math.toRadians(course), kfR_gps)
                    currentHeading = (Math.toDegrees(kfHeading) + 360.0) % 360.0
                    latestHeading = currentHeading
                }

                if (displayMode == 1 && !isConnected && !isGpsConnected) {
                    currentHeading = course
                    latestHeading = course
                }
            }
        }
    }

    private fun runKalmanPredict(gzDeg: Double, dt: Double) {
        kfHeading += Math.toRadians(gzDeg * dt)
        kfP += kfQ
        while (kfHeading > PI) kfHeading -= 2 * PI
        while (kfHeading < -PI) kfHeading += 2 * PI
    }

    private fun runKalmanUpdate(measurementRad: Double, r: Double) {
        var diff = measurementRad - kfHeading
        while (diff > PI) diff -= 2 * PI
        while (diff < -PI) diff += 2 * PI
        
        val k = kfP / (kfP + r)
        kfHeading += k * diff
        kfP = (1.0 - k) * kfP
        
        while (kfHeading > PI) kfHeading -= 2 * PI
        while (kfHeading < -PI) kfHeading += 2 * PI
    }

    private fun startMagCalibration() {
        isCalibratingMag = true; minMx = 1e6; maxMx = -1e6; minMy = 1e6; maxMy = -1e6; minMz = 1e6; maxMz = -1e6
        calDialog = AlertDialog.Builder(this).setTitle("Mag Calibration").setMessage("Rotate 3D...").setPositiveButton("Done") { _, _ ->
            isCalibratingMag = false
            offsetX = (minMx + maxMx) / 2.0; offsetY = (minMy + maxMy) / 2.0; offsetZ = (minMz + maxMz) / 2.0
            val avg = ((maxMx - minMx) / 2.0 + (maxMy - minMy) / 2.0 + (maxMz - minMz) / 2.0) / 3.0
            scaleX = if ((maxMx - minMx) > 2.0) avg / ((maxMx - minMx) / 2.0) else 1.0; scaleY = if ((maxMy - minMy) > 2.0) avg / ((maxMy - minMy) / 2.0) else 1.0; scaleZ = if ((maxMz - minMz) > 2.0) avg / ((maxMz - minMz) / 2.0) else 1.0
            saveCalibration(); Toast.makeText(this, "Mag Calibrated", Toast.LENGTH_SHORT).show()
        }.show()
    }

    private fun startGyroCalibration() {
        isCalibratingGyro = true; gyroSumX = 0.0; gyroSumY = 0.0; gyroSumZ = 0.0; gyroCount = 0
        val pb = ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal).apply { max = 5; progress = 0 }
        calDialog = AlertDialog.Builder(this).setTitle("Gyro Calibration").setMessage("Keep Still...").setView(pb).setCancelable(false).show()
        gyroCalibTimer = object : CountDownTimer(5000, 1000) {
            override fun onTick(ms: Long) { pb.progress = (5000 - ms.toInt()) / 1000 }
            override fun onFinish() {
                isCalibratingGyro = false; if (gyroCount > 0) { gyroOffX = gyroSumX / gyroCount; gyroOffY = gyroSumY / gyroCount; gyroOffZ = gyroSumZ / gyroCount }
                saveCalibration(); calDialog?.dismiss(); Toast.makeText(this@MainActivity, "Gyro Calibrated", Toast.LENGTH_SHORT).show()
            }
        }.start()
    }

    private fun runPidLoop(dt: Double) {
        val current = (currentHeading + gpsHeadingOffset + 360) % 360
        var error = targetHeading!! - current
        if (error > 180) error -= 360 else if (error < -180) error += 360
        
        val activeDb = if (isAutoDeadband) max(3.0, currentSeaState.toDouble()) else deadband
        if (abs(error) < activeDb) { 
            pidIntegral = 0.0
            stopMotors() 
            if (isLogging) writeLog("MOTOR", "OFF,0,0,%.2f,0".format(error))
            return 
        }
        
        pidIntegral = (pidIntegral + error * dt).coerceIn(-500.0, 500.0)
        val deriv = (error - pidLastError) / dt
        val output = pidKp * error + pidKi * pidIntegral + pidKd * deriv
        pidLastError = error

        // Map PID output to Differential Thrust
        val throttle = if (motorMode == 1) 2000 else 750
        val diff = (output * 10).toInt()

        val pVal = (throttle + diff)
        val sVal = (throttle - diff)

        val cmd = if (motorMode == 1) 0x02 else 0x01
        val maxVal = if (motorMode == 1) 10000 else 1000
        val minVal = if (motorMode == 1) 0 else 500

        sendMotorCommand(null, cmd, pVal.coerceIn(minVal, maxVal), sVal.coerceIn(minVal, maxVal))
        if (isLogging) {
            writeLog("MOTOR", "$cmd,$pVal,$sVal,%.2f,%.2f".format(error, output))
        }
    }

    private fun stopMotors() {
        sendMotorCommand(null, 0xFF, 0, 0)
    }

    private fun updateUi(ax: Double, ay: Double, az: Double, mx: Double, my: Double, mz: Double, roll: Double, pitch: Double, rawMag: Double) {
        val deg = (currentHeading + gpsHeadingOffset + 360) % 360
        runOnUiThread {
            binding.txtHeading.text = "%.0f°".format(deg)
            binding.txtStatus.text = "A:%.0f,%.0f,%.0f | M:%.0f,%.0f,%.0f".format(ax, ay, az, mx, my, mz)
            binding.txtSeaState.text = "Sea: $currentSeaState"
            binding.txtGpsInfo.text = "%.1f kn".format(currentGpsSpeedKnots)
            binding.txtGpsOffset.text = "%.1f°".format(gpsHeadingOffset)
            binding.txtRollPitch.text = "R:%.1f° P:%.1f° M:%.0f°".format(roll, pitch, rawMag)
            if (abs(pitch) > 5.0 || abs(roll) > 15.0) binding.txtRollPitch.setTextColor(Color.RED) else binding.txtRollPitch.setTextColor(0xFF616161.toInt())
            updatePidUi()
        }
    }

    private fun updatePidUi() { binding.txtTarget.text = if (targetHeading != null) "Target: %.0f°".format(targetHeading) else "Target: OFF"; binding.txtTarget.setTextColor(if (targetHeading != null) 0xFF1976D2.toInt() else 0xFF757575.toInt()) }
}
