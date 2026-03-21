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
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.*
import android.view.View
import android.view.WindowManager
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.mikewen.ble_compass.databinding.ActivityMainBinding
import java.io.File
import java.io.FileOutputStream
import java.util.*
import kotlin.math.*

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var bluetoothGatt: BluetoothGatt? = null
    private lateinit var prefs: SharedPreferences
    private lateinit var locationManager: LocationManager

    private val SERVICE_UUID = UUID.fromString("0000ae30-0000-1000-8000-00805f9b34fb")
    private val CHAR_UUID = UUID.fromString("0000ae02-0000-1000-8000-00805f9b34fb")

    // --- Hardware Alignment Configuration ---
    // Log Analysis showed: Acc Z is (+), Mag Z is (-). They must match.
    private val ACCT_ROTATED_180 = true  
    private var INVERT_MAG_Z = true      
    private var INVERT_GYRO_Z = true     
    // ----------------------------------------

    private var isCalibratingMag = false
    private var minMx = 1e6; private var maxMx = -1e6
    private var minMy = 1e6; private var maxMy = -1e6
    private var minMz = 1e6; private var maxMz = -1e6
    
    private var offsetX = 0.0; private var offsetY = 0.0; private var offsetZ = 0.0
    private var scaleX = 1.0; private var scaleY = 1.0; private var scaleZ = 1.0

    private var isCalibratingGyro = false
    private var gyroOffsetZ = 0.0; private var gyroSumZ = 0.0; private var gyroCount = 0
    private var gyroCalibTimer: CountDownTimer? = null

    private var kfHeading = 0.0; private var kfP = 1.0
    private val kfQ = 0.0005; private val kfR = 0.1
    private var lastTimestamp = 0L; private var lastUiUpdateTime = 0L

    private val motionBuffer = mutableListOf<Double>()
    private val BUFFER_SIZE = 50 
    private var currentSeaState = 1; private var isKeepScreenOn = false
    private var gpsHeadingOffset = 0.0; private var currentGpsSpeedKnots = 0.0; private var minGpsSpeedKnots = 2.0

    private var logFile: File? = null

    private val scanLauncher = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
        if (result.resultCode == Activity.RESULT_OK) {
            val address = result.data?.getStringExtra("device_address")
            if (address != null) connectToDevice(address)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        prefs = getSharedPreferences("calib_prefs", Context.MODE_PRIVATE)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        loadCalibration()

        logFile = File(getExternalFilesDir(null), "raw_sensor_data.txt")
        if (logFile?.exists() == true) logFile?.delete()

        binding.btnScan.setOnClickListener { checkPermissionsAndScan() }
        binding.btnCalibrateMag.setOnClickListener { if (!isCalibratingMag) startMagCalibration() else stopMagCalibration() }
        binding.btnCalibrateGyro.setOnClickListener { if (!isCalibratingGyro) startGyroCalibration() else stopGyroCalibration() }
        binding.btnResetGps.setOnClickListener { gpsHeadingOffset = 0.0; saveCalibration(); Toast.makeText(this, "GPS Reset", Toast.LENGTH_SHORT).show() }

        binding.btnKeepScreenOn.setOnClickListener {
            isKeepScreenOn = !isKeepScreenOn
            window.apply { if (isKeepScreenOn) addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) else clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON) }
            binding.btnKeepScreenOn.text = "Screen: ${if (isKeepScreenOn) "ON" else "OFF"}"
            binding.btnKeepScreenOn.backgroundTintList = ColorStateList.valueOf(if (isKeepScreenOn) 0xFF4CAF50.toInt() else 0xFF607D8B.toInt())
        }
        setupSpeedSpinner()
    }

    private fun setupSpeedSpinner() {
        val speeds = arrayOf("2.0", "2.5", "3.0", "3.5", "4.0")
        binding.spinnerMinSpeed.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_item, speeds).apply { setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item) }
        binding.spinnerMinSpeed.setSelection(speeds.indexOf(minGpsSpeedKnots.toString()).coerceAtLeast(0))
        binding.spinnerMinSpeed.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) { minGpsSpeedKnots = speeds[pos].toDouble(); saveCalibration() }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }
    }

    private fun loadCalibration() {
        offsetX = prefs.getFloat("mag_off_x", 0.0f).toDouble(); offsetY = prefs.getFloat("mag_off_y", 0.0f).toDouble(); offsetZ = prefs.getFloat("mag_off_z", 0.0f).toDouble()
        scaleX = prefs.getFloat("mag_scale_x", 1.0f).toDouble(); scaleY = prefs.getFloat("mag_scale_y", 1.0f).toDouble(); scaleZ = prefs.getFloat("mag_scale_z", 1.0f).toDouble()
        gyroOffsetZ = prefs.getFloat("gyro_off_z", 0.0f).toDouble()
        gpsHeadingOffset = prefs.getFloat("gps_heading_offset", 0.0f).toDouble()
        minGpsSpeedKnots = prefs.getFloat("min_gps_speed", 2.0f).toDouble()
    }

    private fun saveCalibration() {
        prefs.edit().apply {
            putFloat("mag_off_x", offsetX.toFloat()); putFloat("mag_off_y", offsetY.toFloat()); putFloat("mag_off_z", offsetZ.toFloat())
            putFloat("mag_scale_x", scaleX.toFloat()); putFloat("mag_scale_y", scaleY.toFloat()); putFloat("mag_scale_z", scaleZ.toFloat())
            putFloat("gyro_off_z", gyroOffsetZ.toFloat()); putFloat("gps_heading_offset", gpsHeadingOffset.toFloat()); putFloat("min_gps_speed", minGpsSpeedKnots.toFloat())
            apply()
        }
    }

    private fun checkPermissionsAndScan() {
        val perms = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) { perms.add(Manifest.permission.BLUETOOTH_SCAN); perms.add(Manifest.permission.BLUETOOTH_CONNECT) }
        val missing = perms.filter { ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isEmpty()) { openScanActivity(); startGpsUpdates() } else requestPermissionLauncher.launch(missing.toTypedArray())
    }

    private val requestPermissionLauncher = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { p -> if (p.entries.all { it.value }) { openScanActivity(); startGpsUpdates() } }

    @SuppressLint("MissingPermission")
    private fun startGpsUpdates() { locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 1f, locationListener) }

    private val locationListener = object : LocationListener {
        override fun onLocationChanged(l: Location) {
            if (l.hasSpeed() && l.hasBearing()) {
                currentGpsSpeedKnots = l.speed * 1.94384
                if (currentSeaState <= 2 && currentGpsSpeedKnots >= minGpsSpeedKnots) {
                    var ch = (Math.toDegrees(kfHeading) + 360) % 360
                    var diff = l.bearing - ch
                    while (diff > 180) diff -= 360; while (diff < -180) diff += 360
                    gpsHeadingOffset = 0.995 * gpsHeadingOffset + 0.005 * diff
                    saveCalibration()
                }
            }
        }
        override fun onProviderEnabled(p: String) {}
        override fun onProviderDisabled(p: String) {}
        override fun onStatusChanged(p: String, s: Int, e: Bundle?) {}
    }

    private fun openScanActivity() { scanLauncher.launch(Intent(this, ScanActivity::class.java)) }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(addr: String) {
        val dev = (getSystemService(BLUETOOTH_SERVICE) as BluetoothManager).adapter.getRemoteDevice(addr)
        binding.txtStatus.text = "Status: Connecting..."; bluetoothGatt = dev.connectGatt(this, false, gattCallback)
    }

    private fun startMagCalibration() {
        isCalibratingMag = true; minMx = 1e6; maxMx = -1e6; minMy = 1e6; maxMy = -1e6; minMz = 1e6; maxMz = -1e6
        binding.btnCalibrateMag.text = "STOP MAG CAL"; binding.txtCalibStatus.apply { text = "Rotate 3D (FLIP IT OVER!)"; visibility = View.VISIBLE }
    }

    private fun stopMagCalibration() {
        isCalibratingMag = false; binding.btnCalibrateMag.text = "Cal Mag"; binding.txtCalibStatus.visibility = View.GONE
        if (maxMx > minMx && maxMy > minMy && maxMz > minMz) {
            offsetX = (maxMx + minMx) / 2.0; offsetY = (maxMy + minMy) / 2.0; offsetZ = (maxMz + minMz) / 2.0
            val dx = (maxMx - minMx) / 2.0; val dy = (maxMy - minMy) / 2.0; val dz = (maxMz - minMz) / 2.0
            val avg = (dx + dy + dz) / 3.0
            scaleX = if (dx > 1.0) avg / dx else 1.0; scaleY = if (dy > 1.0) avg / dy else 1.0; scaleZ = if (dz > 1.0) avg / dz else 1.0
            saveCalibration(); Toast.makeText(this, "Mag Cal Saved", Toast.LENGTH_SHORT).show()
        }
    }

    private fun startGyroCalibration() {
        isCalibratingGyro = true; gyroSumZ = 0.0; gyroCount = 0; binding.txtCalibStatus.visibility = View.VISIBLE
        gyroCalibTimer = object : CountDownTimer(5000, 1000) {
            override fun onTick(ms: Long) { binding.txtCalibStatus.text = "Keep Still... ${(ms/1000)+1}" }
            override fun onFinish() { stopGyroCalibration() }
        }.start()
    }

    private fun stopGyroCalibration() {
        isCalibratingGyro = false; gyroCalibTimer?.cancel()
        binding.btnCalibrateGyro.text = "Cal Gyro"; binding.txtCalibStatus.visibility = View.GONE
        if (gyroCount > 0) { gyroOffsetZ = gyroSumZ / gyroCount; saveCalibration(); Toast.makeText(this, "Gyro Cal Saved", Toast.LENGTH_SHORT).show() }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(g: BluetoothGatt, s: Int, n: Int) {
            runOnUiThread {
                if (n == BluetoothProfile.STATE_CONNECTED) { binding.txtStatus.apply { text = "Status: Connected"; setTextColor(0xFF4CAF50.toInt()) }; g.discoverServices() }
                else if (n == BluetoothProfile.STATE_DISCONNECTED) binding.txtStatus.apply { text = "Status: Disconnected"; setTextColor(0xFFF44336.toInt()) }
            }
        }
        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(g: BluetoothGatt, s: Int) {
            val c = g.getService(SERVICE_UUID)?.getCharacteristic(CHAR_UUID)
            if (c != null) { g.setCharacteristicNotification(c, true); val d = c.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")); d.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE; g.writeDescriptor(d) }
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, ch: BluetoothGattCharacteristic) {
            val d = ch.value
            if (d.size >= 20 && d[0] == 0xA1.toByte()) {
                val now = System.currentTimeMillis(); val dt = if (lastTimestamp == 0L) 0.02 else (now - lastTimestamp) / 1000.0; lastTimestamp = now
                var axRaw = ((d[3].toInt() shl 8) or (d[2].toInt() and 0xFF)).toShort().toDouble()
                var ayRaw = ((d[5].toInt() shl 8) or (d[4].toInt() and 0xFF)).toShort().toDouble()
                val azRaw = ((d[7].toInt() shl 8) or (d[6].toInt() and 0xFF)).toShort().toDouble()
                val gzRaw = ((d[13].toInt() shl 8) or (d[12].toInt() and 0xFF)).toShort().toDouble()
                val mxRaw = ((d[15].toInt() shl 8) or (d[14].toInt() and 0xFF)).toShort().toDouble()
                val myRaw = ((d[17].toInt() shl 8) or (d[16].toInt() and 0xFF)).toShort().toDouble()
                val mzRaw = ((d[19].toInt() shl 8) or (d[18].toInt() and 0xFF)).toShort().toDouble()

                if (isCalibratingMag) { minMx = min(minMx, mxRaw); maxMx = max(maxMx, mxRaw); minMy = min(minMy, myRaw); maxMy = max(maxMy, myRaw); minMz = min(minMz, mzRaw); maxMz = max(maxMz, mzRaw) }
                if (isCalibratingGyro) { gyroSumZ += gzRaw; gyroCount++ }

                try { val line = "${now},${axRaw},${ayRaw},${azRaw},${gzRaw},${mxRaw},${myRaw},${mzRaw}\n"; FileOutputStream(logFile, true).use { it.write(line.toByteArray()) } } catch (e: Exception) {}

                // 1. Aligned Reference Frame (NED: X-Forward, Y-Right, Z-Down)
                var ax = axRaw; var ay = ayRaw; var az = azRaw
                if (ACCT_ROTATED_180) { ax = -ax; ay = -ay } // Align Accel PCB to Mag PCB orientation
                
                var mx = (mxRaw - offsetX) * scaleX; var my = (myRaw - offsetY) * scaleY; var mz = (mzRaw - offsetZ) * scaleZ
                if (INVERT_MAG_Z) mz = -mz // Crucial Fix: Log showed Mag Z inverted vs Accel Z
                
                var gzDeg = (gzRaw - gyroOffsetZ) / 64.0; if (INVERT_GYRO_Z) gzDeg = -gzDeg

                // 2. NXP Standard Tilt Compensation
                val amag = sqrt(ax*ax + ay*ay + az*az)
                val nax = ax/amag; val nay = ay/amag; val naz = az/amag
                val phi = atan2(nay, naz) // Roll
                val theta = atan2(-nax, nay * sin(phi) + naz * cos(phi)) // Pitch
                
                val xh = mx * cos(theta) + my * sin(phi) * sin(theta) + mz * cos(phi) * sin(theta)
                val yh = my * cos(phi) - mz * sin(phi)
                val magHeading = atan2(yh, xh) 

                // 3. Kalman Fusion
                kfHeading += Math.toRadians(gzDeg * dt); kfP += kfQ
                var diff = magHeading - kfHeading; while (diff > PI) diff -= 2 * PI; while (diff < -PI) diff += 2 * PI
                kfHeading += (kfP / (kfP + kfR)) * diff; kfP = (1.0 - (kfP / (kfP + kfR))) * kfP
                while (kfHeading > PI) kfHeading -= 2 * PI; while (kfHeading < -PI) kfHeading += 2 * PI

                // 4. Sea State
                val motion = abs(amag / 2048.0 - 1.0) + abs(gzDeg / 100.0)
                motionBuffer.add(motion); if (motionBuffer.size > BUFFER_SIZE) motionBuffer.removeAt(0)
                val varMotion = motionBuffer.map { (it - motionBuffer.average()).pow(2) }.average()
                currentSeaState = when { varMotion < 0.0001 -> 1; varMotion < 0.001 -> 2; varMotion < 0.005 -> 3; varMotion < 0.01 -> 4; varMotion < 0.05 -> 5; varMotion < 0.1 -> 6; varMotion < 0.2 -> 7; varMotion < 0.5 -> 8; else -> 9 }

                if (now - lastUiUpdateTime > 200) {
                    lastUiUpdateTime = now; var deg = (Math.toDegrees(kfHeading) + gpsHeadingOffset + 360) % 360
                    runOnUiThread { binding.txtHeading.text = "%.0f°".format(deg); binding.txtStatus.text = "A:%.0f,%.0f,%.0f | M:%.0f,%.0f,%.0f".format(ax, ay, az, mx, my, mz); binding.txtSeaState.text = "Sea State: $currentSeaState"; binding.txtGpsInfo.text = "GPS: %.1f kn".format(currentGpsSpeedKnots); binding.txtGpsOffset.text = "Offset: %.1f°".format(gpsHeadingOffset) }
                }
            }
        }
    }
}
