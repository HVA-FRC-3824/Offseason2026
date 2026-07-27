package frc.apps.paytoplay

import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.wannaverse.websockets.WebSocketManager
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch

class ws : ViewModel() {

    // --- CONNECTION OPTIONS ---
    // Option A (ADB over USB — recommended): run `adb reverse tcp:5000 tcp:5000` once in a
    // terminal, then localhost on the phone tunnels to your PC via the USB cable.
    //
    // Option B (WiFi): replace with your PC's LAN IP, e.g. "ws://192.168.1.42:5000"
    // Find it by running `ipconfig` and looking for your Wi-Fi adapter's IPv4 address.
    val websocketURL = mutableStateOf("ws://localhost:5000")


    private var wsManager: WebSocketManager = WebSocketManager(websocketURL.value)

    var isLoading = mutableStateOf(false)
    var connected = mutableStateOf(false)
    val messages = mutableStateListOf<String>()

    fun connectToServer() = viewModelScope.launch {
        isLoading.value = true

        wsManager.connect()
        wsManager.incomingMessages.onEach { message ->
            println("Received: $message")
        }.launchIn(CoroutineScope(Dispatchers.Default))
        isLoading.value = false
        connected.value = true
    }

    fun sendMessage(message: String) {
        messages.add("Client: $message")
        wsManager.send(message)
    }

    fun disconnect() {
        connected.value = false
        wsManager.disconnect()
    }

    override fun onCleared() {
        connected.value = false
        super.onCleared()
        wsManager.clear()
    }
}