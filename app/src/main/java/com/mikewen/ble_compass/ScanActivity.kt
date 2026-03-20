package com.mikewen.ble_compass

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.mikewen.ble_compass.databinding.ActivityScanBinding

class ScanActivity : AppCompatActivity() {

    private lateinit var binding: ActivityScanBinding
    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val bluetoothManager = getSystemService(BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothManager.adapter
    }
    private val scanner by lazy { bluetoothAdapter?.bluetoothLeScanner }
    private val handler = Handler(Looper.getMainLooper())
    private var scanning = false

    private val devices = mutableListOf<android.bluetooth.BluetoothDevice>()
    private lateinit var adapter: DeviceAdapter

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityScanBinding.inflate(layoutInflater)
        setContentView(binding.root)

        adapter = DeviceAdapter { device ->
            val intent = Intent().apply {
                putExtra("device_address", device.address)
                putExtra("device_name", device.name)
            }
            setResult(Activity.RESULT_OK, intent)
            finish()
        }

        binding.rvDevices.layoutManager = LinearLayoutManager(this)
        binding.rvDevices.adapter = adapter

        startScan()
    }

    @SuppressLint("MissingPermission")
    private fun startScan() {
        if (scanning) return
        
        devices.clear()
        adapter.notifyDataSetChanged()
        
        handler.postDelayed({
            stopScan()
        }, 10000)

        scanning = true
        binding.progressBar.visibility = View.VISIBLE
        scanner?.startScan(scanCallback)
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        if (!scanning) return
        scanning = false
        binding.progressBar.visibility = View.GONE
        scanner?.stopScan(scanCallback)
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            if (device.name != null && !devices.any { it.address == device.address }) {
                devices.add(device)
                adapter.submitList(devices.toList())
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopScan()
    }

    class DeviceAdapter(private val onClick: (android.bluetooth.BluetoothDevice) -> Unit) :
        RecyclerView.Adapter<DeviceAdapter.ViewHolder>() {

        private var items = listOf<android.bluetooth.BluetoothDevice>()

        fun submitList(list: List<android.bluetooth.BluetoothDevice>) {
            items = list
            notifyDataSetChanged()
        }

        override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
            val view = LayoutInflater.from(parent.context).inflate(R.layout.device_item, parent, false)
            return ViewHolder(view)
        }

        @SuppressLint("MissingPermission")
        override fun onBindViewHolder(holder: ViewHolder, position: Int) {
            val device = items[position]
            holder.name.text = device.name ?: "Unknown"
            holder.address.text = device.address
            holder.itemView.setOnClickListener { onClick(device) }
        }

        override fun getItemCount() = items.size

        class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
            val name: TextView = view.findViewById(R.id.deviceName)
            val address: TextView = view.findViewById(R.id.deviceAddress)
        }
    }
}
