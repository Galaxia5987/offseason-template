package frc.robot.lib.unified_sensor

import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.CURRENT_MODE
import frc.robot.Mode

class UnifiedSensor(
    private val port: Int,
    private val canbus: String = "",
    configuration: CANrangeConfiguration
) {
    private val sensorIO: SensorIO = if (CURRENT_MODE == Mode.REAL) {
        SensorIOReal(port, canbus, configuration )
    } else {
        SensorIOSim()
    }
    val isInRange: Boolean
        get() = sensorIO.inputs.isDetecting

    fun updateInputs() {
        sensorIO.updateInputs()
    }
}