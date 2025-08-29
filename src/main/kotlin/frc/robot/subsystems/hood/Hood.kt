package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import org.littletonrobotics.junction.Logger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.wrist.wristPort


class Hood : SubsystemBase() {
    val hoodMotorConfig = TalonFXConfiguration().apply {
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
    val hoodMotor = UniversalTalonFX(wristPort, "hoodMotor", hoodMotorConfig)
    val positionRequest = PositionVoltage(0.0.degrees)


    fun setPosition(position: Angle): Command {
        return Commands.runOnce({
                hoodMotor.setControl(positionRequest.withPosition(position))
        })
    }

    fun getUp(): Command {
        return Commands.runOnce({ setPosition(HoodPositions.UP.angle) })
    }

    fun getDown(): Command {
        return Commands.runOnce({ setPosition(HoodPositions.Down.angle) })
    }

    fun getToOuttake(): Command {
        return Commands.runOnce({ setPosition(HoodPositions.TAKEOUT.angle) })
    }

    fun getToVertical(): Command {
        return Commands.runOnce({ setPosition(HoodPositions.VERTICAL.angle) })
    }

    override fun periodic() {
        hoodMotor.updateInputs()
        Logger.processInputs(name, hoodMotor.inputs)
    }
}
