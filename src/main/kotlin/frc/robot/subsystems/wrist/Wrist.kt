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
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.seconds
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
                    kP = 45.0
                    kD = 0.3
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
    var setPoint: WristPositions = WristPositions.DOWN

    init {
        wristMotor.reset()
    }

    // TODO: Change setPosition `angle` parameter type to WristPosition
    // TODO: Change setpoint type to WristPositions Enum
    // TODO: Update setpoint
    // TODO: Use this function in the move Commands
    fun setPosition(angle: WristPositions): Command {
        return Commands.runOnce({
            setPoint = angle
            wristMotor.setControl(positionRequest.withPosition(angle.angle)) })
            .andThen(Commands.waitTime(0.2.seconds))
            .andThen(Commands.waitUntil{wristMotor.inputs.current < 2.0.amps})
    }

    fun moveToL1(): Command {
        return setPosition(WristPositions.L1)
    }

    fun moveToL2(): Command {
        return setPosition(WristPositions.L2)
    }

    fun moveToL3(): Command {
        return setPosition(WristPositions.L3)
    }

    fun moveToL4(): Command {
        return setPosition(WristPositions.L4)
    }

    fun moveToCollectCoral(): Command {
        return setPosition(WristPositions.FEEDER)
    }
    override fun periodic() {
        wristMotor.updateInputs()
        Logger.processInputs(name, wristMotor.inputs)
        Logger.recordOutput("Wrist/setPoint", setPoint)
    }
}
