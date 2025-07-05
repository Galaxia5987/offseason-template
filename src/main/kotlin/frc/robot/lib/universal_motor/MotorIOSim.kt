package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.extensions.toDistance

class MotorIOSim(
    private val momentOfInertia: MomentOfInertia,
    override val config: TalonFXConfiguration,
    private val controller: PIDController,
    private val gearRatio: Double,
    private val radius: Distance
) : MotorIO {
    override val inputs = LoggedMotorInputs()
    private val motor =
        TalonFXSim(
            1,
            1.0,
            momentOfInertia.`in`(Units.KilogramSquareMeters),
            1.0,
            TalonType.KRAKEN_FOC
        )

    init {
        motor.setController(controller)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        motor.setControl(controlRequest)
    }

    override fun updateInputs() {
        motor.update(Timer.getTimestamp())
        inputs.current = motor.appliedCurrent
        inputs.position = Rotations.of(motor.position)
        inputs.voltage = motor.appliedVoltage
        inputs.distance = Rotations.of(motor.position).toDistance(radius, gearRatio)
    }
}
