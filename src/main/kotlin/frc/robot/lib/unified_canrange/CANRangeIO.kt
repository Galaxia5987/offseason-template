package frc.robot.lib.unified_canrange

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.extensions.cm

interface CANRangeIO {
    val inputs: SensorInputs

    fun updateInputs() {}

    @Logged
    open class SensorInputs {
        var distance: Distance = 0.cm
        var isDetecting: Boolean = false
    }
}
