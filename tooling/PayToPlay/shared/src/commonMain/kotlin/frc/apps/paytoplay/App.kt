package frc.apps.paytoplay

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.style.LineBreak
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.atruedev.kmpnfc.adapter.NfcAdapter
import com.atruedev.kmpnfc.ndef.NdefRecord
import com.atruedev.kmpnfc.reader.AndroidScanMode
import com.atruedev.kmpnfc.reader.ReaderOptions

@Composable
@Preview
fun App() {
    MaterialTheme {

        var error by remember { mutableStateOf<String?>("") }

        var numOfTags by remember { mutableStateOf(0) }

        // Remember the adapter so it isn't re-created on every recomposition
        val adapter = remember { NfcAdapter() }

        val connection = remember { ws() }

        var tagData by remember { mutableStateOf("Waiting for tag...") }
        var isScanning by remember { mutableStateOf(false) }

        val sendData = remember {
            { ->
                try {
                    connection.sendMessage(numOfTags.toString())
                } catch (e: Exception) {
                    error = e.message
                }
            }
        }

        // When isScanning flips to true, start collecting tags until canceled
        LaunchedEffect(isScanning) {
            if (!isScanning) return@LaunchedEffect
            tagData = "Hold NFC card near phone..."
            try {
                adapter.tags(
                    ReaderOptions(
                        // ReaderMode is simpler and more reliable than ForegroundDispatch —
                        // it doesn't require BroadcastReceivers or PendingIntents
                        androidScanMode = AndroidScanMode.ReaderMode
                    )
                ).collect { tag ->
                    val result = tag.use {
                        val ndef = it.readNdef() ?: return@use "Tag found but no NDEF data"

                        val parts = mutableListOf<String>()
                        ndef.records.forEach { record ->
                            when (record) {
                                is NdefRecord.Uri -> parts.add("URL: ${record.uri}")
                                is NdefRecord.Text -> parts.add("Text: ${record.text}")
                                is NdefRecord.MimeMedia -> parts.add("MIME: ${record.mimeType}")
                                else -> parts.add("Other record type: $record")
                            }
                        }
                        if (parts.isEmpty()) "Tag read but no records found"
                        else parts.joinToString("\n")
                    }
                    // Both state writes happen here, on the main thread
                    numOfTags++
                    tagData = result
                    sendData()
                }
            } catch (e: Exception) {
                tagData = "Error: ${e.message}"
                error = e.message
                isScanning = false
            }
        }

        Box(modifier = Modifier.fillMaxSize().padding(16.dp), contentAlignment = Alignment.Center) {
            Column(modifier = Modifier.align(Alignment.Center)) {
                Button(onClick = { isScanning = !isScanning }) {
                    Text(if (isScanning) "Stop Scanning" else "Start Scanning")
                }
                Button(onClick = sendData) {
                    Text("Send Data")
                }
                Button(onClick = {
                    try {
                        connection.connectToServer()
                    }
                    catch (e: Exception) {
                        error = e.message
                    }
                }) {
                    Text("Start Server")
                }
                Text("NFC State: ${adapter.state.value}")
                Text("Data: $tagData")
                Text("Tags found: $numOfTags")
                LineBreak
                if (!error.isNullOrEmpty()) Text("LastError: $error", color = Color.Red)
            }
        }
    }
}