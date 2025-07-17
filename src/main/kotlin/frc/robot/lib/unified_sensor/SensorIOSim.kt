package frc.robot.lib.unified_sensor

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class SensorIOSim: SensorIO {
    override val inputs = SensorIO.SensorInputs()
    private val isDetecting = LoggedNetworkBoolean("IsDetecting", false)

    override fun updateInputs() {
        inputs.isDetecting = isDetecting.get()
    }
}