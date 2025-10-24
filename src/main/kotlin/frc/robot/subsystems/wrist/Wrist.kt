package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.elevator.ratio
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

@AutoLogOutput(key = "Wrist/mechanism")
var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Wrist", 3.0, 2.0)
val ligament =
    root.append(LoggedMechanismLigament2d("WristLigament", 0.25, 90.0))

object Wrist: SubsystemBase() {

    val motor1: UniversalTalonFX =
        UniversalTalonFX(0, config = config, gearRatio = ratio)
    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    var setPoint: Angle = 0.0.degrees

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce(
            {
                setPoint = angle
                motor1.setControl(positionVoltageRequest.withPosition(angle))
            }
        )
    }

    fun setAnglePosition(position: WristPositions): Command{
        return setAngle(position.angle)
    }

    override fun periodic() {
        motor1.updateInputs()
        ligament.setAngle(motor1.inputs.position[degrees])
        Logger.processInputs("Wrist", motor1.inputs)
        Logger.recordOutput("Wrist/setpoint", setPoint)
        Logger.recordOutput("Subsystems/Wrist/Ligament", mechanism)
    }
}

