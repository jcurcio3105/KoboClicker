package com.example.individualcounter

/**
 * Represents one row of data coming from the Arduino.
 *
 * Columns:
 *  c1..c5  = button columns sent from the device
 *  timestamp = time received by ANDROID (System.currentTimeMillis)
 */
data class BatchEntry(
    val c1: Int,
    val c2: Int,
    val c3: Int,
    val c4: Int,
    val c5: Int,
    val timestamp: Long
) {
    companion object {

        /**
         * Parses a CSV line from the Arduino.
         * We expect at least 5 columns:
         *   c1,c2,c3,c4,c5,timestamp_ms,bt_sent,sd_sent
         *
         * We IGNORE the Arduino timestamp and generate our own.
         */
        fun fromCSVLine(line: String): BatchEntry? {
            val parts = line.split(",")

            // Must have at least 5 values
            if (parts.size < 5) return null

            return try {
                val c1 = parts[0].trim().toInt()
                val c2 = parts[1].trim().toInt()
                val c3 = parts[2].trim().toInt()
                val c4 = parts[3].trim().toInt()
                val c5 = parts[4].trim().toInt()

                // Always use ANDROID timestamp, ignore Arduino millis()
                val phoneTimestamp = System.currentTimeMillis()

                BatchEntry(c1, c2, c3, c4, c5, phoneTimestamp)

            } catch (e: Exception) {
                null
            }
        }
    }

    /**
     * CSV-friendly row for export/sharing.
     * Format: Timestamp,c1,c2,c3,c4,c5
     */
    fun toCSVLine(): String = "$timestamp,$c1,$c2,$c3,$c4,$c5"
}

