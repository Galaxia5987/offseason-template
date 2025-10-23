package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Hood : SubsystemBase() {
    private val motorHood: UniversalTalonFX = UniversalTalonFX(3)
    private val positionReq: PositionVoltage = PositionVoltage(0.0)
    private var setPoint: Angle = 0.0.degrees
    val config: TalonFXConfiguration = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().apply {
            Inverted = InvertedValue.Clockwise_Positive
            NeutralMode = NeutralModeValue.Brake
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = 20.0
            SupplyCurrentLimit = 10.0
        }
        Slot0 = Slot0Configs().apply {
            kP = 0.0
            kD = 0.0
        }
        Feedback = FeedbackConfigs().apply {
            SensorToMechanismRatio = GEAR_RATIO
        }
    }

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            setPoint = angle
            motorHood.setControl(positionReq.withPosition(angle))
        })
    }

    fun setAngle1(): Command {
        return setAngle(Angles.ANGLE1.angle)
    }

    fun setAngle2(): Command{
        return setAngle(Angles.ANGLE2.angle)
    }
    fun setAngle3(): Command{
        return setAngle(Angles.ANGLE3.angle)
    }

    fun setAngle4(): Command{
        return setAngle(Angles.ANGLE4.angle)
    }



    override fun periodic() {

        motorHood.updateInputs()
        Logger.processInputs("Hood", motorHood.inputs)
        Logger.recordOutput("Hood/setAngle", setPoint)

    }
}