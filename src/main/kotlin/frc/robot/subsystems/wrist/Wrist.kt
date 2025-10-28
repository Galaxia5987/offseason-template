package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Wrist : SubsystemBase() {
    val motorConfig =
        TalonFXConfiguration().apply {
            MotorOutput =
                MotorOutputConfigs().apply {
                    NeutralMode = NeutralModeValue.Coast
                    Inverted = InvertedValue.CounterClockwise_Positive
                }
            Feedback =
                FeedbackConfigs().apply {
                    RotorToSensorRatio = 1.0
                    SensorToMechanismRatio = GEAR_RATIO
                }
            Slot0 =
                Slot0Configs().apply {
                    kP = 1.0
                    kD = 0.25
                    //                    kP = 45.0
                    //                    kD = 0.3
                    GravityType = GravityTypeValue.Arm_Cosine
                    StaticFeedforwardSign =
                        StaticFeedforwardSignValue.UseClosedLoopSign
                }
            CurrentLimits =
                CurrentLimitsConfigs().apply {
                    StatorCurrentLimitEnable = true
                    SupplyCurrentLimitEnable = true
                    StatorCurrentLimit = 20.0
                    SupplyCurrentLimit = 40.0
                }
        }
    val wristMotor =
        UniversalTalonFX(
            wristPort,
            config = motorConfig,
            gearRatio = GEAR_RATIO
        )
    val positionRequest = PositionVoltage(0.0.degrees)
    var setPoint = 0.0.degrees
    var angle = { wristMotor.inputs.position }

    fun setPosition(angle: Angle): Command {
        return Commands.runOnce({
            wristMotor.setControl(positionRequest.withPosition(angle))
        })
    }

    init {
        wristMotor.reset()
    }

    fun moveToInTaking(): Command {
        return Commands.runOnce({
            setPoint = WristPositions.InTaking.angle
            wristMotor.setControl(
                positionRequest.withPosition(WristPositions.InTaking.angle)
            )
        })
    }

    fun moveToL1(): Command {
        return Commands.runOnce({
            setPoint = WristPositions.L1.angle
            wristMotor.setControl(
                positionRequest.withPosition(WristPositions.L1.angle)
            )
        })
    }

    fun moveToL2(): Command {
        return Commands.runOnce({
            setPoint = WristPositions.L2.angle
            wristMotor.setControl(
                positionRequest.withPosition(WristPositions.L2.angle)
            )
        })
    }

    fun moveToL3(): Command {
        return Commands.runOnce({
            setPoint = WristPositions.L3.angle
            wristMotor.setControl(
                positionRequest.withPosition(WristPositions.L3.angle)
            )
        })
    }

    fun moveToL4(): Command {
        return Commands.runOnce({
            setPoint = WristPositions.L4.angle
            wristMotor.setControl(
                positionRequest.withPosition(WristPositions.L4.angle)
            )
        })
    }

    override fun periodic() {
        wristMotor.updateInputs()
        Logger.processInputs(name, wristMotor.inputs)
        Logger.recordOutput("Wrist/setPoint", setPoint)
    }
}
