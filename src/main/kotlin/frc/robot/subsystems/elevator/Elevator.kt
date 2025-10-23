package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SlotConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

object Elevator: SubsystemBase() {

    private val motor1: UniversalTalonFX =
        UniversalTalonFX(0, config = config, gearRatio = ratio, linearSystemWheelDiameter = wheelDiameter)
    private val motor2: UniversalTalonFX =
        UniversalTalonFX(1, config = config, gearRatio = ratio)

    init {
        motor2.setControl(Follower(0, false))
    }

    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    private var setPoint: Distance = 0.0.meters

    fun setLevel(level: ElevatorPositions): Command {
        return Commands.runOnce(
            {
                setPoint = level.position
                motor1.setControl(positionVoltageRequest.withPosition(level.position.toAngle(wheelDiameter, ratio)))
            }
        )
    }

    override fun periodic() {
        motor1.updateInputs()
        Logger.processInputs("Elevator", motor1.inputs)
        Logger.recordOutput("Elevator/setpoint", setPoint)
    }
}