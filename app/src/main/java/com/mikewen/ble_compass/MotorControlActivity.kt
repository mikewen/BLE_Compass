package com.mikewen.ble_compass

import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.SeekBar
import androidx.appcompat.app.AppCompatActivity
import com.mikewen.ble_compass.databinding.ActivityMotorControlBinding
import kotlin.math.abs

class MotorControlActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMotorControlBinding
    private val handler = Handler(Looper.getMainLooper())
    
    // --- Step Configuration (Easy to change here) ---
    private val ESC_STEP_SMALL = 2
    private val ESC_STEP_LARGE = 10
    private val BLDC_STEP_SMALL = 100
    private val BLDC_STEP_LARGE = 500

    private var portValue = 0
    private var stbdValue = 0
    // ------------------------------------------------

    private val updateRunnable = object : Runnable {
        override fun run() {
            updateSensorDisplay()
            handler.postDelayed(this, 200)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMotorControlBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.btnBack.setOnClickListener { finish() }

        setupMotorControls()
        handler.post(updateRunnable)
    }

    private fun getMax() = if (MainActivity.motorMode == 0) 500 else 10000

    private fun adjustPort(delta: Int) {
        portValue = (portValue + delta).coerceIn(0, getMax())
        updateValues()
    }

    private fun adjustStbd(delta: Int) {
        stbdValue = (stbdValue + delta).coerceIn(0, getMax())
        updateValues()
    }

    private fun adjustBoth(delta: Int) {
        portValue = (portValue + delta).coerceIn(0, getMax())
        stbdValue = (stbdValue + delta).coerceIn(0, getMax())
        updateValues()
    }
    private fun setupMotorControls() {

        if (MainActivity.motorMode == 1) {
            binding.rbBldc.isChecked = true
            setBldcMode()
        } else {
            binding.rbEsc.isChecked = true
            setEscMode()
        }

        binding.rgMotorMode.setOnCheckedChangeListener { _, checkedId ->
            if (checkedId == R.id.rbEsc) {
                MainActivity.motorMode = 0
                setEscMode()
            } else {
                MainActivity.motorMode = 1
                setBldcMode()
            }
        }

        // PORT
        binding.btnPortPP.setOnClickListener { adjustPort(getLargeStep()); sendCommand() }
        binding.btnPortP.setOnClickListener { adjustPort(getSmallStep()); sendCommand() }
        binding.btnPortM.setOnClickListener { adjustPort(-getSmallStep()); sendCommand() }
        binding.btnPortMM.setOnClickListener { adjustPort(-getLargeStep()); sendCommand() }

        // STBD
        binding.btnStbdPP.setOnClickListener { adjustStbd(getLargeStep()); sendCommand() }
        binding.btnStbdP.setOnClickListener { adjustStbd(getSmallStep()); sendCommand() }
        binding.btnStbdM.setOnClickListener { adjustStbd(-getSmallStep()); sendCommand() }
        binding.btnStbdMM.setOnClickListener { adjustStbd(-getLargeStep()); sendCommand() }

        // BOTH
        binding.btnBothPP.setOnClickListener {
            adjustBoth(getLargeStep())
            sendCommand()
        }
        binding.btnBothP.setOnClickListener {
            adjustBoth(getSmallStep())
            sendCommand()
        }
        binding.btnBothM.setOnClickListener {
            adjustBoth(-getSmallStep())
            sendCommand()
        }
        binding.btnBothMM.setOnClickListener {
            adjustBoth(-getLargeStep())
            sendCommand()
        }

        binding.btnStop.setOnClickListener {
            portValue = 0
            stbdValue = 0
            updateValues()
            MainActivity.sendMotorCommand(MainActivity.imuGattInstance, 0xFF, 0, 0)
        }
    }

    private fun getSmallStep() = if (MainActivity.motorMode == 0) ESC_STEP_SMALL else BLDC_STEP_SMALL
    private fun getLargeStep() = if (MainActivity.motorMode == 0) ESC_STEP_LARGE else BLDC_STEP_LARGE

    private fun setEscMode() {
        portValue = 0
        stbdValue = 0
        updateValues()
    }

    private fun setBldcMode() {
        portValue = 0
        stbdValue = 0
        updateValues()
    }

    private fun adjustSeekBar(sb: SeekBar, delta: Int) {
        sb.progress = (sb.progress + delta).coerceIn(0, sb.max)
        updateValues()
    }

    private fun updateValues() {
        val isEsc = MainActivity.motorMode == 0

        val portDisplay = if (isEsc) portValue + 500 else portValue
        val stbdDisplay = if (isEsc) stbdValue + 500 else stbdValue

        binding.txtPortVal.text = portDisplay.toString()
        binding.txtStarboardVal.text = stbdDisplay.toString()
    }

    private fun sendCommand() {
        val isEsc = MainActivity.motorMode == 0
        val cmd = if (isEsc) 0x01 else 0x02

        val portSend = if (isEsc) portValue + 500 else portValue
        val stbdSend = if (isEsc) stbdValue + 500 else stbdValue

        MainActivity.sendMotorCommand(
            MainActivity.imuGattInstance,
            cmd,
            portSend,
            stbdSend
        )
    }

    private fun updateSensorDisplay() {
        binding.txtHeading.text = "%.0f°".format(MainActivity.latestHeading)
        val speedStr = "%.1f kn".format(MainActivity.latestSpeed)

        val spannable = android.text.SpannableString(speedStr)

        // make "kn" smaller
        val unitStart = speedStr.indexOf("kn")
        if (unitStart >= 0) {
            spannable.setSpan(
                android.text.style.RelativeSizeSpan(0.2f), // 50% size
                unitStart,
                speedStr.length,
                android.text.Spannable.SPAN_EXCLUSIVE_EXCLUSIVE
            )
        }

        binding.txtSpeed.text = spannable
        binding.txtRollPitch.text = "R: %.1f° P: %.1f°".format(MainActivity.latestRoll, MainActivity.latestPitch)
        
        if (abs(MainActivity.latestPitch) > 5.0 || abs(MainActivity.latestRoll) > 15.0) {
            binding.txtRollPitch.setTextColor(Color.RED)
        } else {
            binding.txtRollPitch.setTextColor(0xFF616161.toInt())
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        handler.removeCallbacks(updateRunnable)
    }
}
