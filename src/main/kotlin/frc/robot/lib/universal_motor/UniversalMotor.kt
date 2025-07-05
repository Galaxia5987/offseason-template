package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.lib.universal_motor.LoggedMotorInputs
import frc.robot.lib.universal_motor.MotorIO
import frc.robot.lib.universal_motor.MotorIOReal
import frc.robot.lib.universal_motor.MotorIOSim

// If Motor is not a linearSubsystem (For example Elevator) No need for radius
class UniversalMotor(
    port: Int,
    canbus: String = "",
    config: TalonFXConfiguration = TalonFXConfiguration(),
    momentOfInertia: MomentOfInertia,
    gearRatio: Double = 1.0,
    radius: Distance = Meters.zero()
) {
    private val motorIO: MotorIO
    val inputs: LoggedMotorInputs
        get() = motorIO.inputs

    init {
        motorIO = if (CURRENT_MODE == Mode.REAL) MotorIOReal(
            port = port,
            canbus,
            config,
            gearRatio,
            radius
        ) else {
            val slot0Config: Slot0Configs = config.Slot0
            val controller = PIDController(slot0Config.kP, slot0Config.kI, slot0Config.kD)

            MotorIOSim(momentOfInertia, config, controller, gearRatio, radius)
        }
    }

    fun setControl(control: ControlRequest) = motorIO.setRequest(control)

    fun updateInputs() = motorIO.updateInputs()
}
