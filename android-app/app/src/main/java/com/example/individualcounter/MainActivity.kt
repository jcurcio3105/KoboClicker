package com.example.individualcounter

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import kotlinx.coroutines.flow.collectLatest
import java.io.File
import java.text.SimpleDateFormat
import java.util.*

class MainActivity : ComponentActivity() {
    private lateinit var bleManager: BLEManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        bleManager = BLEManager(this)

        if (!hasBlePermissions(this)) {
            requestBlePermissions()
        }

        setContent {
            Surface(modifier = Modifier.fillMaxSize()) {
                DynamicColumnCounterScreen(bleManager)
            }
        }
    }

    fun hasBlePermissions(context: Context): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.BLUETOOTH_SCAN
            ) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(
                        context,
                        Manifest.permission.BLUETOOTH_CONNECT
                    ) == PackageManager.PERMISSION_GRANTED
        } else {
            ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED
        }
    }

    fun requestBlePermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requestPermissions(
                arrayOf(
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT
                ),
                101
            )
        } else {
            requestPermissions(
                arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                101
            )
        }
    }
}

@Composable
fun DynamicColumnCounterScreen(bleManager: BLEManager) {
    val context = LocalContext.current

    // Observe BLE connection state
    val connectionState by bleManager.connectionState.collectAsState()

    // 5 data columns
    var headers by remember {
        mutableStateOf(listOf("Col1", "Col2", "Col3", "Col4", "Col5"))
    }
    val batches = remember { mutableStateListOf<BatchEntry>() }
    var showScanDialog by remember { mutableStateOf(false) }

    // Collect BLE data
    LaunchedEffect(Unit) {
        bleManager.incomingBatch.collectLatest { batch ->
            Log.d("UI", "Received batch in UI: size=${batch.size}")
            batches.addAll(batch)
        }
    }

    // Show a Toast when connected
    LaunchedEffect(connectionState) {
        if (connectionState == ConnectionState.CONNECTED) {
            Toast.makeText(context, "Bluetooth connected", Toast.LENGTH_SHORT).show()
        }
    }

    val totals = headers.indices.map { col ->
        batches.sumOf { entry ->
            when (col) {
                0 -> entry.c1
                1 -> entry.c2
                2 -> entry.c3
                3 -> entry.c4
                4 -> entry.c5
                else -> 0
            }
        }
    }

    val connectionText = when (connectionState) {
        ConnectionState.CONNECTED -> "Connected"
        ConnectionState.CONNECTING -> "Connecting..."
        ConnectionState.DISCONNECTED -> "Disconnected"
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(WindowInsets.systemBars.asPaddingValues())
            .padding(16.dp)
    ) {
        // BLE connect button
        Button(onClick = { showScanDialog = true }) {
            Text("Connect BLE")
        }

        // Simple connection status indicator
        Spacer(Modifier.height(8.dp))
        Text("Bluetooth status: $connectionText")

        if (showScanDialog) {
            ScanDialog(context, bleManager) { showScanDialog = false }
        }

        Spacer(Modifier.height(16.dp))

        Text("Live CSV Viewer", style = MaterialTheme.typography.headlineSmall)
        Spacer(Modifier.height(16.dp))

        Text("Rows received: ${batches.size}")
        Spacer(Modifier.height(8.dp))

        // Table headers
        Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
            Text("Time", modifier = Modifier.width(100.dp))

            headers.forEachIndexed { index, header ->
                var editingText by remember(header) { mutableStateOf(header) }

                BasicTextField(
                    value = editingText,
                    onValueChange = {
                        editingText = it
                        headers = headers.toMutableList().apply { set(index, it) }
                    },
                    textStyle = TextStyle(color = MaterialTheme.colorScheme.onSurface),
                    modifier = Modifier
                        .width(80.dp)
                        .padding(4.dp)
                )
            }
        }

        Divider(Modifier.padding(vertical = 8.dp))

        // Display rows dynamically
        LazyColumn(
            modifier = Modifier.weight(1f),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            items(batches) { entry ->
                Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                    val timestampStr = SimpleDateFormat(
                        "HH:mm:ss",
                        Locale.getDefault()
                    ).format(Date(entry.timestamp))
                    Text(timestampStr, modifier = Modifier.width(100.dp))

                    listOf(entry.c1, entry.c2, entry.c3, entry.c4, entry.c5).forEach {
                        Text(it.toString(), modifier = Modifier.width(80.dp))
                    }
                }
            }
        }

        Divider(Modifier.padding(vertical = 8.dp))

        Text("Totals:", style = MaterialTheme.typography.titleMedium)
        Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
            headers.forEachIndexed { index, header ->
                Text("$header: ${totals[index]}")
            }
        }

        Spacer(Modifier.height(16.dp))

        Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
            Button(
                onClick = {
                    val csv = batchesToCsv(batches, headers)
                    exportCsvToDevice(
                        context,
                        "batches_${System.currentTimeMillis()}.csv",
                        csv
                    )
                },
                enabled = batches.isNotEmpty()
            ) {
                Text("Save CSV")
            }

            Button(
                onClick = {
                    exportCsvViaShare(context, batches, headers)
                },
                enabled = batches.isNotEmpty()
            ) {
                Text("Share CSV")
            }
        }
    }
}

// --- CSV Helpers (unchanged) ---
fun batchesToCsv(batches: List<BatchEntry>, headers: List<String>): String {
    val headerRow = "Timestamp," + headers.joinToString(",")
    val rows = batches.joinToString("\n") {
        "${it.timestamp},${it.c1},${it.c2},${it.c3},${it.c4},${it.c5}"
    }
    return "$headerRow\n$rows"
}

fun exportCsvToDevice(context: Context, fileName: String, data: String) {
    try {
        val file = File(context.getExternalFilesDir(null), fileName)
        file.writeText(data)
        Toast.makeText(context, "CSV saved: ${file.absolutePath}", Toast.LENGTH_LONG).show()
    } catch (e: Exception) {
        Toast.makeText(context, "Error saving CSV: ${e.message}", Toast.LENGTH_LONG).show()
    }
}

fun exportCsvViaShare(context: Context, batches: List<BatchEntry>, headers: List<String>) {
    val csv = batchesToCsv(batches, headers)
    val file = File(context.cacheDir, "batches.csv")
    file.writeText(csv)

    val uri = androidx.core.content.FileProvider.getUriForFile(
        context,
        "${context.packageName}.provider",
        file
    )

    val shareIntent = android.content.Intent(android.content.Intent.ACTION_SEND).apply {
        type = "text/csv"
        putExtra(android.content.Intent.EXTRA_STREAM, uri)
        addFlags(android.content.Intent.FLAG_GRANT_READ_URI_PERMISSION)
    }

    context.startActivity(
        android.content.Intent.createChooser(
            shareIntent,
            "Share CSV"
        )
    )
}


