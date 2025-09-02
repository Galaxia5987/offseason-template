package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.toUnit
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Wrist : SubsystemBase() {
    val wristMotorConfig = TalonFXConfiguration().apply {
        Slot0.apply {
            kP = 0.3
            kD - 0.0
        }
        MotorOutput = MotorOutputConfigs().apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.Clockwise_Positive
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = 30.0
            SupplyCurrentLimit = 60.0
        }
        SoftwareLimitSwitch = SoftwareLimitSwitchConfigs().apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold = forwardLimits
            ReverseSoftLimitThreshold = reverseLimits
        }
    }

    val wristMotor = UniversalTalonFX(wristPort, "wristMotor", wristMotorConfig)
    val positionRequest = PositionVoltage(0.0.degrees)

    fun addToPosition(position: Angle): Command {
        return Commands.run({
            val currentPosition = wristMotor.inputs.position
            positionRequest.withPosition(currentPosition + position)
            wristMotor.setControl(positionRequest)
            })
    }

    fun moveToL1() : Command {
        return Commands.runOnce({
            wristMotor.setControl(positionRequest.withPosition(WristPositions.L1.wristAngle))
        })
    }

    fun moveToL2 () : Command {
        return Commands.runOnce({
            wristMotor.setControl(positionRequest.withPosition(WristPositions.L2.wristAngle))
        })
    }

    fun moveToL3() : Command {
        return Commands.runOnce({
            wristMotor.setControl(positionRequest.withPosition(WristPositions.L3.wristAngle))
        })
    }

    fun moveToL4() : Command {
        return Commands.runOnce({
            wristMotor.setControl(positionRequest.withPosition(WristPositions.L4.wristAngle))
        })
    }

    override fun periodic() {
        wristMotor.updateInputs()
        Logger.processInputs(name, wristMotor.inputs)
    }
}
