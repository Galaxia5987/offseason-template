package frc.robot.lib.unified_sensor

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.wpilibj2.command.Command

class SensorIOReal
    (private val port: Int,
     private val canbus: String = "",
     configuration: CANrangeConfiguration
            ) : SensorIO {
    override val inputs = SensorIO.SensorInputs()

    private val CANrange = CANrange(port, canbus)

    init {
        CANrange.configurator.apply(
            configuration
        )
    }

    override fun updateInputs() {
        inputs.distance = CANrange.distance.value
        inputs.isDetecting = CANrange.isDetected.value
    }
}
