package frc.robot.lib.unified_canrange

import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.CURRENT_MODE
import frc.robot.Mode

class UnifiedCANRange(
    private val port: Int,
    private val canbus: String = "",
    configuration: CANrangeConfiguration
) {
    private val sensorIO: CANRangeIO = if (CURRENT_MODE == Mode.REAL) {
        CANRangeIOReal(port, canbus, configuration)
    } else {
        CANRangeIOSim()
    }
    val isInRange: Boolean
        get() = sensorIO.inputs.isDetecting

    fun updateInputs() {
        sensorIO.updateInputs()
    }
}