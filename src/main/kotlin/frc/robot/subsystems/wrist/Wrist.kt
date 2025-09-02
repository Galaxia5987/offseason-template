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
    }

    val wristMotor = UniversalTalonFX(wristPort, "wristMotor", wristMotorConfig)
    val positionRequest = PositionVoltage(0.0.degrees)
    val Limits = SoftwareLimitSwitchConfigs().apply {
        ForwardSoftLimitEnable = true
        ReverseSoftLimitEnable = true
        ForwardSoftLimitThreshold = forwardLimits
        ReverseSoftLimitThreshold = reverseLimits
    }
    init {
        wristMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        wristMotorConfig.MotorOutput.withInverted(
            InvertedValue.Clockwise_Positive
        )
        wristMotorConfig.CurrentLimits.withStatorCurrentLimit(12.0)
            .withSupplyCurrentLimit(12.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
    }

    fun addToPosition(position: Angle): Command {
        return Commands.run({
            val currentPosition = wristMotor.inputs.position
            positionRequest.withPosition(currentPosition + position)
            wristMotor.setControl(positionRequest)
            })
    }

    override fun periodic() {
        wristMotor.updateInputs()
        Logger.processInputs(name, wristMotor.inputs)
    }
}
