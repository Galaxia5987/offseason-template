package frc.robot.lib.unified_canrange

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class CANRangeIOSim: CANRangeIO {
    override val inputs = CANRangeIO.SensorInputs()
    private val isDetecting = LoggedNetworkBoolean("IsDetecting", false)

    override fun updateInputs() {
        inputs.isDetecting = isDetecting.get()
    }
}