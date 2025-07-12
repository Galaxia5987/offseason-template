package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.lib.extensions.m

// If Motor is not a linearSubsystem (For example Elevator or a linear intake) No need for diameter
class UniversalMotor(
    port: Int,
    canbus: String = "",
    config: TalonFXConfiguration = TalonFXConfiguration(),
    momentOfInertia: MomentOfInertia,
    gearRatio: Double = 1.0,
    linearSystemWheelDiameter: Distance = 0.m,
) {
    private val motorIO: MotorIO
    val inputs: LoggedMotorInputs
        get() = motorIO.inputs

    init {
        motorIO =
            if (CURRENT_MODE == Mode.REAL)
                MotorIOReal(port, canbus, config, gearRatio, linearSystemWheelDiameter)
            else {
                MotorIOSim(
                    momentOfInertia,
                    config,
                    gearRatio,
                    linearSystemWheelDiameter
                )
            }
    }

    fun setControl(control: ControlRequest) = motorIO.setRequest(control)

    fun updateInputs() = motorIO.updateInputs()
}
