package com.example.individualcounter

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Toast
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat

/**
 * Popup dialog to scan BLE devices and connect
 */
@SuppressLint("MissingPermission") // we check permission before scanning
@Composable
fun ScanDialog(
    context: Context,
    bleManager: BLEManager,
    onDismiss: () -> Unit
) {
    val bluetoothManager =
        context.getSystemService(Context.BLUETOOTH_SERVICE) as? android.bluetooth.BluetoothManager
    val bluetoothAdapter: BluetoothAdapter? = bluetoothManager?.adapter

    var devices by remember { mutableStateOf(listOf<BluetoothDevice>()) }
    var scanning by remember { mutableStateOf(false) }

    val handler = remember { Handler(Looper.getMainLooper()) }

    val scanCallback = remember {
        object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult?) {
                result?.device?.let { device ->
                    if (!devices.contains(device)) {
                        devices = devices + device
                    }
                    Log.d(
                        "BLE_SCAN",
                        "Found device: ${device.name ?: "Unnamed"} (${device.address})"
                    )
                }
            }

            override fun onBatchScanResults(results: MutableList<ScanResult>?) {
                results?.forEach { r ->
                    r.device?.let { device ->
                        if (!devices.contains(device)) {
                            devices = devices + device
                        }
                        Log.d(
                            "BLE_SCAN",
                            "Found device in batch: ${device.name ?: "Unnamed"} (${device.address})"
                        )
                    }
                }
            }

            override fun onScanFailed(errorCode: Int) {
                Toast.makeText(context, "BLE scan failed: $errorCode", Toast.LENGTH_SHORT).show()
                Log.e("BLE_SCAN", "Scan failed with error: $errorCode")
            }
        }
    }

    // Start scan when dialog appears
    LaunchedEffect(Unit) {
        // 1. Check BLE support
        val hasBle =
            context.packageManager.hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)
        if (!hasBle) {
            Toast.makeText(
                context,
                "This device does not support Bluetooth Low Energy",
                Toast.LENGTH_LONG
            ).show()
            return@LaunchedEffect
        }

        // 2. Check adapter present & enabled
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled) {
            Toast.makeText(
                context,
                "Please turn ON Bluetooth and try again",
                Toast.LENGTH_LONG
            ).show()
            return@LaunchedEffect
        }

        // 3. Check scan permission on Android 12+
        val hasScanPermission =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                ContextCompat.checkSelfPermission(
                    context,
                    Manifest.permission.BLUETOOTH_SCAN
                ) == PackageManager.PERMISSION_GRANTED
            } else {
                true
            }

        if (!hasScanPermission) {
            Toast.makeText(
                context,
                "Bluetooth scan permission not granted",
                Toast.LENGTH_SHORT
            ).show()
            return@LaunchedEffect
        }

        try {
            Log.d("BLE_SCAN", "Starting BLE scan…")
            bluetoothAdapter.bluetoothLeScanner?.startScan(scanCallback)
            scanning = true
        } catch (e: SecurityException) {
            Log.e("BLE_SCAN", "SecurityException starting scan", e)
            Toast.makeText(
                context,
                "Cannot start BLE scan (missing permission)",
                Toast.LENGTH_SHORT
            ).show()
            return@LaunchedEffect
        }

        // Stop after 10 seconds
        handler.postDelayed({
            try {
                Log.d("BLE_SCAN", "Stopping BLE scan after timeout")
                bluetoothAdapter.bluetoothLeScanner?.stopScan(scanCallback)
            } catch (e: SecurityException) {
                Log.e("BLE_SCAN", "SecurityException stopping scan", e)
            }
            scanning = false
        }, 10_000)
    }

    AlertDialog(
        onDismissRequest = onDismiss,
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        },
        title = { Text("Select BLE Device") },
        text = {
            Column(modifier = Modifier.fillMaxWidth()) {
                if (scanning) {
                    Text("Scanning...", style = MaterialTheme.typography.bodyMedium)
                    Spacer(Modifier.height(8.dp))
                }

                if (!scanning && devices.isEmpty()) {
                    Text(
                        "No BLE devices found.\n\n" +
                                "Make sure your counter is powered, close to the tablet, " +
                                "and actually using BLE (not classic Bluetooth)."
                    )
                }

                LazyColumn(modifier = Modifier.height(200.dp)) {
                    items(devices) { device ->
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable {
                                    bleManager.connectToDevice(device)
                                    Toast.makeText(
                                        context,
                                        "Connecting to ${device.name ?: device.address}",
                                        Toast.LENGTH_SHORT
                                    ).show()
                                    onDismiss()
                                }
                                .padding(vertical = 8.dp, horizontal = 4.dp)
                        ) {
                            Text(device.name ?: "Unnamed", modifier = Modifier.weight(1f))
                            Text(device.address)
                        }
                    }
                }
            }
        }
    )
}


