package com.example.individualcounter

import android.Manifest
import android.bluetooth.*
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import androidx.core.content.ContextCompat
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import java.util.*
import kotlin.collections.ArrayList

/**
 * Handles BLE connection, receiving rows, batching 1 row before emitting.
 */
class BLEManager(private val context: Context) {

    companion object {
        private const val TAG = "BLEManager"

        // BLE UART service and characteristic UUIDs
        val UART_SERVICE_UUID: UUID =
            UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        val TX_CHAR_UUID: UUID =
            UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E") // Peripheral → Phone
        val RX_CHAR_UUID: UUID =
            UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E") // Phone → Peripheral

        private const val BATCH_SIZE = 1
    }

    private val bluetoothManager: BluetoothManager =
        context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val adapter: BluetoothAdapter? = bluetoothManager.adapter

    private var gatt: BluetoothGatt? = null
    private var txChar: BluetoothGattCharacteristic? = null

    private val tempBatch = mutableListOf<BatchEntry>()

    // IMPORTANT: replay = 1 so the latest batch is delivered
    // even if the collector started slightly later.
    private val _incomingBatch = MutableSharedFlow<List<BatchEntry>>(
        replay = 1,
        extraBufferCapacity = 64
    )
    val incomingBatch: SharedFlow<List<BatchEntry>>
        get() = _incomingBatch

    // --- Connection state for UI ---
    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ConnectionState>
        get() = _connectionState

    // ---------- Connect ----------
    fun connectToDevice(device: BluetoothDevice) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ContextCompat.checkSelfPermission(
                    context,
                    Manifest.permission.BLUETOOTH_CONNECT
                ) != PackageManager.PERMISSION_GRANTED
            ) {
                Log.w(TAG, "BLUETOOTH_CONNECT permission missing")
                return
            }
        }

        _connectionState.value = ConnectionState.CONNECTING

        gatt = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
        } else {
            device.connectGatt(context, false, gattCallback)
        }
    }

    fun disconnect() {
        gatt?.disconnect()
        gatt?.close()
        gatt = null
        txChar = null
        tempBatch.clear()
        _connectionState.value = ConnectionState.DISCONNECTED
        Log.d(TAG, "Disconnected and cleared state")
    }

    // ---------- Send message ----------
    fun sendMessage(message: String) {
        txChar?.let { char ->
            char.value = message.toByteArray(Charsets.UTF_8)
            gatt?.writeCharacteristic(char)
            Log.d(TAG, "Sent message via BLE: $message")
        } ?: Log.w(TAG, "TX characteristic not available, cannot send message")
    }

    // ---------- GATT Callback ----------
    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                Log.d(TAG, "Connected → discovering services")
                _connectionState.value = ConnectionState.CONNECTED
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                Log.d(TAG, "Disconnected")
                _connectionState.value = ConnectionState.DISCONNECTED
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d(TAG, "Service discovery complete")

                val service: BluetoothGattService? = gatt.getService(UART_SERVICE_UUID)

                if (service != null) {
                    txChar = service.getCharacteristic(TX_CHAR_UUID)
                    val rxChar = service.getCharacteristic(RX_CHAR_UUID)

                    Log.d(TAG, "UART service found")
                    Log.d(TAG, "TX characteristic: ${txChar?.uuid}")
                    Log.d(TAG, "RX characteristic: ${rxChar?.uuid}")

                    // Enable notifications on TX (Peripheral → Phone)
                    gatt.setCharacteristicNotification(txChar, true)

                    val descriptor = txChar?.getDescriptor(
                        UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
                    )
                    descriptor?.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    gatt.writeDescriptor(descriptor)

                    Log.d(TAG, "UART notifications enabled on TX characteristic")
                } else {
                    Log.e(TAG, "UART service not found")
                }
            } else {
                Log.e(TAG, "Service discovery failed with status: $status")
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            if (characteristic.uuid == TX_CHAR_UUID) {
                val rawBytes = characteristic.value
                val line = rawBytes.toString(Charsets.UTF_8).trim()

                Log.d(TAG, "Received raw bytes: ${rawBytes.joinToString(",")}")
                Log.d(TAG, "Received string: $line")

                handleIncomingLine(line)
            }
        }
    }

    // ---------- Handle incoming row ----------
    private fun handleIncomingLine(line: String) {
        val entry = BatchEntry.fromCSVLine(line)

        if (entry != null) {
            tempBatch.add(entry)

            Log.d(
                TAG,
                "Parsed row: c1=${entry.c1}, c2=${entry.c2}, c3=${entry.c3}, c4=${entry.c4}, c5=${entry.c5}, ts=${entry.timestamp}"
            )

            if (tempBatch.size >= BATCH_SIZE) {
                val batchCopy = ArrayList(tempBatch)
                tempBatch.clear()
                val success = _incomingBatch.tryEmit(batchCopy)
                Log.d(TAG, "Emitted batch of size ${batchCopy.size}, tryEmit=$success")
            }
        } else {
            Log.w(TAG, "Failed to parse line into BatchEntry: '$line'")
        }
    }
}

// Simple connection state enum for UI
enum class ConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED
}



