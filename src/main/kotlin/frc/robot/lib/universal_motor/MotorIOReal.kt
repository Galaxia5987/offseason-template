package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.toDistance

class MotorIOReal(
    private val port: Int,
    private val canBus: String,
    override val config: TalonFXConfiguration,
    private val gearRatio: Double,
    private val radius: Distance
) : MotorIO {
    override val inputs = LoggedMotorInputs()
    private val motor = TalonFX(port, canBus)

    init {
        motor.configurator.apply(config)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        motor.setControl(controlRequest)
    }

    override fun updateInputs() {
        inputs.current = motor.supplyCurrent.value
        inputs.position = motor.position.value
        inputs.voltage = motor.motorVoltage.value
        inputs.distance = motor.position.value.toDistance(radius, gearRatio)
    }
}
