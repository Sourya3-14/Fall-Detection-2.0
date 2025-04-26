package com.example.bluetoothapp

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import java.io.InputStream
import java.io.OutputStream
import java.util.*
import kotlin.concurrent.thread

class MainActivity : AppCompatActivity() {

    private val TAG = "MainActivity"
    private val REQUEST_PERMISSION_CODE = 1

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private var inputStream: InputStream? = null

    private lateinit var connectButton: Button
    private lateinit var sendDataButton: Button
    private lateinit var phoneInput: EditText
    private lateinit var wifiIdInput: EditText
    private lateinit var wifiPassInput: EditText
    private lateinit var receivedText: TextView
    private lateinit var bpmTextView: TextView
    private lateinit var spo2TextView: TextView
    private lateinit var decisionTextView: TextView

    private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        connectButton = findViewById(R.id.connectButton)
        sendDataButton = findViewById(R.id.sendDataButton)
        phoneInput = findViewById(R.id.phoneInput)
        wifiIdInput = findViewById(R.id.wifiIdInput)
        wifiPassInput = findViewById(R.id.wifiPassInput)
        receivedText = findViewById(R.id.receivedText)
        bpmTextView = findViewById(R.id.bpmTextView)
        spo2TextView = findViewById(R.id.spo2TextView)
        decisionTextView = findViewById(R.id.decisionTextView)

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth not supported!", Toast.LENGTH_SHORT).show()
            finish()
        }

        checkBluetoothPermissions()

        connectButton.setOnClickListener {
            selectDeviceAndConnect()
        }

        sendDataButton.setOnClickListener {
            val phone = phoneInput.text.toString()
            val wifiId = wifiIdInput.text.toString()
            val wifiPass = wifiPassInput.text.toString()
            sendUserDataToESP32(phone, wifiId, wifiPass)
        }
    }

    private fun checkBluetoothPermissions() {
        val permissions = arrayOf(
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.BLUETOOTH_CONNECT,
            Manifest.permission.BLUETOOTH_SCAN
        )

        ActivityCompat.requestPermissions(this, permissions, REQUEST_PERMISSION_CODE)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_PERMISSION_CODE) {
            if (grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
                Toast.makeText(this, "Bluetooth permissions granted!", Toast.LENGTH_SHORT).show()
            } else {
                Toast.makeText(this, "Permissions denied. App may not work properly.", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun selectDeviceAndConnect() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
            != PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(this, "Bluetooth permission not granted!", Toast.LENGTH_SHORT).show()
            return
        }

        val pairedDevices: Set<BluetoothDevice>? = bluetoothAdapter?.bondedDevices

        if (pairedDevices.isNullOrEmpty()) {
            Toast.makeText(this, "No paired devices found", Toast.LENGTH_SHORT).show()
            return
        }

        val deviceNames = pairedDevices.map { it.name ?: it.address }.toTypedArray()
        val devicesList = pairedDevices.toList()

        runOnUiThread {
            val builder = androidx.appcompat.app.AlertDialog.Builder(this)
            builder.setTitle("Select a Bluetooth device")

            builder.setItems(deviceNames) { _, which ->
                val selectedDevice = devicesList[which]
                connectToDevice(selectedDevice)
            }

            builder.setNegativeButton("Cancel") { dialog, _ ->
                dialog.dismiss()
            }

            builder.show()
        }
    }

    private fun connectToDevice(device: BluetoothDevice) {
        thread {
            var connected = false
            var attempts = 0
            while (!connected && attempts < 5) {
                try {
                    if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                        != PackageManager.PERMISSION_GRANTED
                    ) {
                        runOnUiThread {
                            Toast.makeText(this, "Bluetooth permission not granted!", Toast.LENGTH_SHORT).show()
                        }
                        return@thread
                    }

                    bluetoothSocket = device.createRfcommSocketToServiceRecord(SPP_UUID)
                    bluetoothSocket?.connect()

                    outputStream = bluetoothSocket?.outputStream
                    inputStream = bluetoothSocket?.inputStream

                    connected = true

                    runOnUiThread {
                        Toast.makeText(this, "Connected to ${device.name}", Toast.LENGTH_SHORT).show()
                    }

                    listenForIncomingData()

                } catch (e: SecurityException) {
                    Log.e(TAG, "Security exception: ${e.message}")
                    break
                } catch (e: Exception) {
                    attempts++
                    Log.e(TAG, "Connection attempt $attempts failed: ${e.message}")
                    bluetoothSocket?.close()
                    Thread.sleep(2000)
                }
            }

            if (!connected) {
                runOnUiThread {
                    Toast.makeText(this, "Failed to connect after multiple attempts.", Toast.LENGTH_LONG).show()
                }
            }
        }
    }

    private fun sendUserDataToESP32(phone: String, wifiId: String, wifiPass: String) {
        val defaultPhone = "0123456789"
        val defaultWifiId = "MyWiFi"
        val defaultWifiPass = "password"

        val finalPhone = if (phone.isNotBlank()) phone else defaultPhone
        val finalWifiId = if (wifiId.isNotBlank()) wifiId else defaultWifiId
        val finalWifiPass = if (wifiPass.isNotBlank()) wifiPass else defaultWifiPass

        val message = "$finalPhone,$finalWifiId,$finalWifiPass\n"

        thread {
            try {
                outputStream?.write(message.toByteArray())
                runOnUiThread {
                    Toast.makeText(this, "Sent: $message", Toast.LENGTH_SHORT).show()
                }
            } catch (e: Exception) {
                Log.e(TAG, "Sending failed: ${e.message}")
                runOnUiThread {
                    Toast.makeText(this, "Failed to send data", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }
    private fun listenForIncomingData() {
        thread {
            try {
                val buffer = ByteArray(1024)
                var bytesRead: Int

                while (true) {
                    bytesRead = inputStream?.read(buffer) ?: -1
                    if (bytesRead > 0) {
                        val receivedMessage = String(buffer, 0, bytesRead).trim()

                        val parts = receivedMessage.split(",")
                        if (parts.size >= 3) {
                            val bpm = parts[0]
                            val spo2 = parts[1]
                            val decision = parts[2]

                            runOnUiThread {
                                bpmTextView.text = "BPM: $bpm"
                                spo2TextView.text = "SpO2: $spo2 %"
                                decisionTextView.text = "Decision: $decision"
                            }
                        }
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Receiving failed: ${e.message}")
            }
        }
    }
    override fun onDestroy() {
        super.onDestroy()
        bluetoothSocket?.close()
    }
}
