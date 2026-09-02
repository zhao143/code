package com.example.car_control

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.Context
import android.os.Build
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.BufferedReader
import java.io.InputStream
import java.io.InputStreamReader
import java.io.OutputStream
import java.util.UUID

/**
 * 经典蓝牙 SPP 串口管理器。
 *
 * STM32 USART2 接的是 HC-05/HC-06/HC-08 这一类“串口透传”模块时，手机看到的是
 * 一个经典蓝牙串口连接。这里不主动扫描设备，只列出 Android 系统设置里已经配对
 * 的设备，避免增加定位权限和扫描流程对初学者造成的干扰。
 */
class BluetoothSerialManager(context: Context) {

    companion object {
        /**
         * 蓝牙串口服务的标准 UUID。
         *
         * HC-05、HC-06 等经典 SPP 模块通常使用这个 UUID。若后续更换为 BLE 模块，
         * 不能继续使用这一套 RFCOMM 连接，需要单独实现 GATT 版本。
         */
        private val SPP_UUID: UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

        private const val HOLD_REFRESH_MS = 100L
        private const val MAX_LOG_LINES = 100
    }

    private val appContext = context.applicationContext
    private val bluetoothAdapter: BluetoothAdapter? =
        appContext.getSystemService(BluetoothManager::class.java)?.adapter
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private var socket: android.bluetooth.BluetoothSocket? = null
    private var input: InputStream? = null
    private var output: OutputStream? = null
    private var connectionJob: Job? = null
    private var readJob: Job? = null
    private var holdJob: Job? = null
    private val connectionLock = Any()
    private var connectionGeneration = 0L

    private val _pairedDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val pairedDevices: StateFlow<List<BluetoothDevice>> = _pairedDevices.asStateFlow()

    private val _connectionState = MutableStateFlow("未连接")
    val connectionState: StateFlow<String> = _connectionState.asStateFlow()

    private val _logs = MutableStateFlow<List<String>>(emptyList())
    val logs: StateFlow<List<String>> = _logs.asStateFlow()

    /**
     * 读取已经配对的设备列表。
     *
     * Android 12 及以上调用该 API 前必须由 Activity 申请 BLUETOOTH_CONNECT；
     * 这里仍然捕获 SecurityException，避免用户拒绝权限后 App 直接崩溃。
     */
    @SuppressLint("MissingPermission")
    fun refreshPairedDevices() {
        try {
            if (bluetoothAdapter == null) {
                addLog("本机没有蓝牙适配器")
                _pairedDevices.value = emptyList()
                return
            }
            _pairedDevices.value = bluetoothAdapter.bondedDevices
                .toList()
                .sortedWith(compareBy(String.CASE_INSENSITIVE_ORDER) { it.name ?: it.address })
        } catch (error: SecurityException) {
            addLog("没有蓝牙权限，请允许“附近设备”权限")
        } catch (error: Exception) {
            addLog("读取配对设备失败：${error.message ?: error.javaClass.simpleName}")
        }
    }

    /**
     * 连接指定的经典蓝牙串口设备。
     *
     * 连接、读数据全部运行在 IO 协程，避免阻塞 Compose 主线程。连接成功后会
     * 启动持续读行任务，底盘返回的 OK、BUSY、STATUS 等文本会显示在日志区域。
     */
    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        disconnect()
        val generation = synchronized(connectionLock) {
            connectionGeneration += 1L
            connectionGeneration
        }
        _connectionState.value = "正在连接 ${device.name ?: device.address}"
        addLog("连接设备：${device.name ?: device.address}")

        connectionJob = scope.launch {
            var candidateSocket: android.bluetooth.BluetoothSocket? = null
            try {
                if (bluetoothAdapter == null) {
                    throw IllegalStateException("本机没有蓝牙适配器")
                }

                bluetoothAdapter.cancelDiscovery()
                candidateSocket = device.createRfcommSocketToServiceRecord(SPP_UUID)
                candidateSocket!!.connect()
                synchronized(connectionLock) {
                    if (generation != connectionGeneration) {
                        candidateSocket?.close()
                        return@launch
                    }
                    socket = candidateSocket
                    input = candidateSocket!!.inputStream
                    output = candidateSocket!!.outputStream
                }
                _connectionState.value = "已连接：${device.name ?: device.address}"
                addLog("连接成功")
                sendLineInternal("status")
                readJob = launchReadLoop(candidateSocket!!.inputStream, generation)
            } catch (error: SecurityException) {
                candidateSocket?.close()
                if (isCurrentGeneration(generation)) {
                    _connectionState.value = "权限不足"
                    addLog("蓝牙权限不足：请允许“附近设备”权限")
                    closeSocket()
                }
            } catch (error: CancellationException) {
                candidateSocket?.close()
                throw error
            } catch (error: Exception) {
                candidateSocket?.close()
                if (isCurrentGeneration(generation)) {
                    _connectionState.value = "连接失败"
                    addLog("连接失败：${error.message ?: error.javaClass.simpleName}")
                    closeSocket()
                }
            }
        }
    }

    /**
     * 关闭当前连接，并确保电机收到停车命令。
     */
    fun disconnect() {
        stopHold()
        synchronized(connectionLock) {
            connectionGeneration += 1L
        }
        connectionJob?.cancel()
        connectionJob = null
        sendLineInternal("stop")
        closeSocket()
        _connectionState.value = "未连接"
    }

    /**
     * 发送一条带 CRLF 结尾的文本命令。
     *
     * 底盘蓝牙协议按换行分隔命令，例如：`pwm 300 300`、`stop`、`status`。
     */
    fun sendLine(line: String) {
        scope.launch {
            sendLineInternal(line.trim())
        }
    }

    /**
     * 开始按住控制。
     *
     * 运动命令每 100ms 重发一次，与 STM32 的 800ms 蓝牙租约超时配合使用。即使
     * 手机或连接突然异常，底盘也会在没有刷新命令后自动停车。
     */
    fun startHold(command: String) {
        stopHold()
        holdJob = scope.launch {
            while (isActive) {
                sendLineInternal(command)
                delay(HOLD_REFRESH_MS)
            }
        }
    }

    /**
     * 松开方向键时立刻停车。
     */
    fun stopHold() {
        holdJob?.cancel()
        holdJob = null
        if (isConnected()) {
            scope.launch {
                sendLineInternal("stop")
            }
        }
    }

    /**
     * 释放管理器持有的协程和蓝牙资源。
     */
    fun close() {
        stopHold()
        closeSocket()
        scope.cancel()
    }

    private fun isConnected(): Boolean =
        socket?.isConnected == true && output != null

    private fun launchReadLoop(stream: InputStream, generation: Long): Job =
        scope.launch {
            try {
                val reader = BufferedReader(InputStreamReader(stream, Charsets.UTF_8))
                while (isActive) {
                    val line = withContext(Dispatchers.IO) { reader.readLine() }
                        ?: break
                    if (line.isNotBlank()) {
                        addLog("底盘：$line")
                    }
                }
                addLog("蓝牙串口已断开")
            } catch (error: CancellationException) {
                throw error
            } catch (error: Exception) {
                addLog("读取蓝牙数据失败：${error.message ?: error.javaClass.simpleName}")
            } finally {
                if (isCurrentGeneration(generation)) {
                    _connectionState.value = "未连接"
                    closeSocket()
                }
            }
        }

    private fun isCurrentGeneration(generation: Long): Boolean =
        synchronized(connectionLock) {
            generation == connectionGeneration
        }

    private fun sendLineInternal(line: String) {
        if (line.isBlank()) {
            return
        }
        val currentOutput = output ?: return
        try {
            currentOutput.write("$line\r\n".toByteArray(Charsets.UTF_8))
            currentOutput.flush()
            addLog("手机：$line")
        } catch (error: Exception) {
            addLog("发送失败：${error.message ?: error.javaClass.simpleName}")
            closeSocket()
        }
    }

    @Synchronized
    private fun closeSocket() {
        readJob?.cancel()
        readJob = null
        try {
            input?.close()
        } catch (_: Exception) {
        }
        try {
            output?.close()
        } catch (_: Exception) {
        }
        try {
            socket?.close()
        } catch (_: Exception) {
        }
        input = null
        output = null
        socket = null
    }

    private fun addLog(message: String) {
        val old = _logs.value
        _logs.value = (old + message).takeLast(MAX_LOG_LINES)
    }
}
