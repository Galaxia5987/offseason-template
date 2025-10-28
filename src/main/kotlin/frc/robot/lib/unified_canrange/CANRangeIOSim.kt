package frc.robot.lib.unified_canrange

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class CANRangeIOSim(subsystemName: String) : CANRangeIO {
    override val inputs = CANRangeIO.SensorInputs()
    private val isDetecting =
        LoggedNetworkBoolean("/Tuning/$subsystemName/IsDetecting", false)

    override fun updateInputs() {
        inputs.isDetecting = isDetecting.get()
    }
}
