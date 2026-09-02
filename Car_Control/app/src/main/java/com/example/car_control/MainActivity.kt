package com.example.car_control

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Slider
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.car_control.ui.theme.Car_ControlTheme

class MainActivity : ComponentActivity() {

    private lateinit var bluetoothManager: BluetoothSerialManager
    private var bluetoothPermissionGranted by mutableStateOf(false)

    private val bluetoothPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            bluetoothPermissionGranted = granted
            if (granted) {
                bluetoothManager.refreshPairedDevices()
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        bluetoothManager = BluetoothSerialManager(this)
        bluetoothPermissionGranted = hasBluetoothConnectPermission(this)
        setContent {
            Car_ControlTheme {
                CarControlScreen(
                    bluetoothManager = bluetoothManager,
                    hasPermission = bluetoothPermissionGranted,
                    onRequestPermission = {
                        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                            bluetoothPermissionLauncher.launch(
                                Manifest.permission.BLUETOOTH_CONNECT
                            )
                        } else {
                            bluetoothManager.refreshPairedDevices()
                        }
                    }
                )
            }
        }
    }

    override fun onDestroy() {
        bluetoothManager.close()
        super.onDestroy()
    }
}

private fun hasBluetoothConnectPermission(context: Context): Boolean {
    return Build.VERSION.SDK_INT < Build.VERSION_CODES.S ||
        ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED
}

/**
 * 手机控制首页。
 *
 * 页面保持单页结构：上面管理蓝牙连接，中间是速度和方向控制，下面是底盘
 * 返回的日志。第一次调试时可以直接看到命令是否发出、底盘是否回复。
 */
@Composable
private fun CarControlScreen(
    bluetoothManager: BluetoothSerialManager,
    hasPermission: Boolean,
    onRequestPermission: () -> Unit
) {
    val devices by bluetoothManager.pairedDevices.collectAsState()
    val connectionState by bluetoothManager.connectionState.collectAsState()
    val logs by bluetoothManager.logs.collectAsState()
    var speed by remember { mutableFloatStateOf(300f) }

    DisposableEffect(bluetoothManager) {
        onDispose { bluetoothManager.close() }
    }

    LaunchedEffect(hasPermission) {
        if (hasPermission) {
            bluetoothManager.refreshPairedDevices()
        }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = {
                    Column {
                        Text("Car Control", fontWeight = FontWeight.Bold)
                        Text(
                            text = connectionState,
                            style = MaterialTheme.typography.labelSmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant
                        )
                    }
                }
            )
        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(horizontal = 16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            if (!hasPermission) {
                Surface(
                    modifier = Modifier.fillMaxWidth(),
                    shape = RoundedCornerShape(8.dp),
                    color = MaterialTheme.colorScheme.errorContainer
                ) {
                    Column(Modifier.padding(12.dp)) {
                        Text(
                            "需要“附近设备”权限才能连接蓝牙串口模块。",
                            color = MaterialTheme.colorScheme.onErrorContainer
                        )
                        Spacer(Modifier.height(8.dp))
                        Button(onClick = onRequestPermission) {
                            Text("授予蓝牙权限")
                        }
                    }
                }
            }

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text("已配对设备", style = MaterialTheme.typography.titleMedium)
                TextButton(onClick = {
                    if (hasPermission) {
                        bluetoothManager.refreshPairedDevices()
                    } else {
                        onRequestPermission()
                    }
                }) {
                    Text("刷新")
                }
            }

            if (devices.isEmpty()) {
                Text(
                    "没有找到已配对设备。请先在系统蓝牙设置中配对 HC-05/HC-06。",
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            } else {
                LazyColumn(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(112.dp),
                    verticalArrangement = Arrangement.spacedBy(6.dp)
                ) {
                    items(devices, key = { it.address }) { device ->
                        Surface(
                            modifier = Modifier.fillMaxWidth(),
                            shape = RoundedCornerShape(8.dp),
                            tonalElevation = 1.dp
                        ) {
                            Row(
                                modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
                                verticalAlignment = Alignment.CenterVertically
                            ) {
                                Column(Modifier.weight(1f)) {
                                    Text(device.name ?: "未命名蓝牙设备")
                                    Text(
                                        device.address,
                                        style = MaterialTheme.typography.labelSmall,
                                        color = MaterialTheme.colorScheme.onSurfaceVariant
                                    )
                                }
                                OutlinedButton(onClick = {
                                    bluetoothManager.connect(device)
                                }) {
                                    Text("连接")
                                }
                            }
                        }
                    }
                }
            }

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                OutlinedButton(
                    modifier = Modifier.weight(1f),
                    onClick = { bluetoothManager.sendLine("status") }
                ) {
                    Text("读取状态")
                }
                OutlinedButton(
                    modifier = Modifier.weight(1f),
                    onClick = { bluetoothManager.disconnect() }
                ) {
                    Text("断开并停车")
                }
            }

            HorizontalDivider()

            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text("速度 ${speed.toInt()}", style = MaterialTheme.typography.titleMedium)
                Spacer(Modifier.width(12.dp))
                Slider(
                    modifier = Modifier.weight(1f),
                    value = speed,
                    onValueChange = { speed = it },
                    valueRange = 100f..700f,
                    steps = 11
                )
            }

            Text(
                "按住方向键运动，松开立即停车",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )

            DirectionPad(
                enabled = connectionState.startsWith("已连接"),
                speed = speed.toInt(),
                onStart = bluetoothManager::startHold,
                onStop = bluetoothManager::stopHold
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Button(
                    modifier = Modifier.weight(1f),
                    onClick = { bluetoothManager.sendLine("enable") }
                ) {
                    Text("使能")
                }
                Button(
                    modifier = Modifier.weight(1f),
                    onClick = { bluetoothManager.sendLine("stop") }
                ) {
                    Text("停车")
                }
                OutlinedButton(
                    modifier = Modifier.weight(1f),
                    onClick = { bluetoothManager.sendLine("clear") }
                ) {
                    Text("清故障")
                }
            }

            Text("通信日志", style = MaterialTheme.typography.titleMedium)
            LazyColumn(
                modifier = Modifier
                    .fillMaxWidth()
                    .weight(1f)
                    .border(
                        width = 1.dp,
                        color = MaterialTheme.colorScheme.outlineVariant,
                        shape = RoundedCornerShape(8.dp)
                    )
                    .padding(8.dp),
                reverseLayout = true
            ) {
                items(logs.asReversed()) { line ->
                    Text(
                        line,
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(vertical = 2.dp),
                        fontSize = 12.sp
                    )
                }
            }
        }
    }
}

@Composable
private fun DirectionPad(
    enabled: Boolean,
    speed: Int,
    onStart: (String) -> Unit,
    onStop: () -> Unit
) {
    val speedText = speed.coerceIn(100, 700)
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        HoldDirectionButton(
            label = "前",
            command = "pwm $speedText $speedText",
            enabled = enabled,
            onStart = onStart,
            onStop = onStop
        )
        Row(horizontalArrangement = Arrangement.spacedBy(20.dp)) {
            HoldDirectionButton(
                label = "左",
                command = "pwm -$speedText $speedText",
                enabled = enabled,
                onStart = onStart,
                onStop = onStop
            )
            HoldDirectionButton(
                label = "停",
                command = "stop",
                enabled = enabled,
                onStart = { onStart("stop") },
                onStop = onStop
            )
            HoldDirectionButton(
                label = "右",
                command = "pwm $speedText -$speedText",
                enabled = enabled,
                onStart = onStart,
                onStop = onStop
            )
        }
        HoldDirectionButton(
            label = "后",
            command = "pwm -$speedText -$speedText",
            enabled = enabled,
            onStart = onStart,
            onStop = onStop
        )
    }
}

/**
 * 可按住的方向按钮。
 *
 * 使用 pointerInput 监听按下和释放，而不是只监听一次点击。手指持续按住就
 * 持续刷新运动命令，松手就停车。
 */
@Composable
private fun HoldDirectionButton(
    label: String,
    command: String,
    enabled: Boolean,
    onStart: (String) -> Unit,
    onStop: () -> Unit
) {
    var pressed by remember { mutableStateOf(false) }
    val background = when {
        !enabled -> MaterialTheme.colorScheme.surfaceVariant
        pressed -> MaterialTheme.colorScheme.primary
        else -> MaterialTheme.colorScheme.primaryContainer
    }
    val foreground = when {
        !enabled -> MaterialTheme.colorScheme.onSurfaceVariant
        pressed -> MaterialTheme.colorScheme.onPrimary
        else -> MaterialTheme.colorScheme.onPrimaryContainer
    }

    Box(
        modifier = Modifier
            .size(76.dp)
            .clip(RoundedCornerShape(12.dp))
            .background(background)
            .pointerInput(enabled, command) {
                detectTapGestures(
                    onPress = {
                        if (!enabled) {
                            return@detectTapGestures
                        }
                        pressed = true
                        onStart(command)
                        try {
                            tryAwaitRelease()
                        } finally {
                            pressed = false
                            onStop()
                        }
                    }
                )
            },
        contentAlignment = Alignment.Center
    ) {
        Text(
            label,
            color = foreground,
            fontSize = 24.sp,
            fontWeight = FontWeight.Bold,
            textAlign = TextAlign.Center
        )
    }
}
