package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.wrist.Wrist
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

@AutoLogOutput(key = "Elevator/mechanism")
private var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Elevator", 3.0, 2.0)
private val ligament =
    root.append(LoggedMechanismLigament2d("ElevatorLigament", 0.25, 90.0))

object Elevator : SubsystemBase() {
    val motor1: UniversalTalonFX =
        UniversalTalonFX(
            0,
            config = config,
            gearRatio = ratio,
            linearSystemWheelDiameter = wheelDiameter
        )
    private val motor2: UniversalTalonFX =
        UniversalTalonFX(1, config = config, gearRatio = ratio)

    init {
        motor2.setControl(Follower(0, false))
    }

    val atSetPoint = Trigger { Elevator.setPoint == Elevator.motor1.inputs.position }

    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    var setPoint: Distance = 0.0.meters

    fun setLevel(level: ElevatorPositions): Command {
        return Commands.runOnce({
            setPoint = level.position
            motor1.setControl(
                positionVoltageRequest.withPosition(
                    level.position.toAngle(wheelDiameter, ratio)
                )
            )
        })
    }

    override fun periodic() {
        motor1.updateInputs()
        ligament.length = motor1.inputs.distance[meters]
        Logger.processInputs("Elevator", motor1.inputs)
        Logger.recordOutput("Elevator/setpoint", setPoint)
        Logger.recordOutput("Subsystems/Elevator/Ligament", mechanism)
    }
}
