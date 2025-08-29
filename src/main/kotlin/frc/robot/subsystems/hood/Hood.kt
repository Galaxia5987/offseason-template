package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import org.littletonrobotics.junction.Logger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.wrist.wristPort


class Hood : SubsystemBase() {
    val hoodMotorConfig = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.Clockwise_Positive
        }
        Slot0 = Slot0Configs().apply {
            kP = 0.3
            kD = 0.0
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = 30.0
            SupplyCurrentLimit = 60.0
        }
    }
    val hoodMotor = UniversalTalonFX(wristPort, config = hoodMotorConfig)
    val positionRequest = PositionVoltage(0.0.degrees)
    val voltageRequest = VoltageOut(0.0.volts)


    fun setPosition(position: Angle): Command {
        return Commands.runOnce({
                hoodMotor.setControl(positionRequest.withPosition(position))
        })
    }
    fun stopMotor(): Command{
        return Commands.runOnce({hoodMotor.setControl(voltageRequest.withOutput(0.0))})
    }
    fun getUp(): Command {
        return setPosition(HoodPositions.UP.angle)
    }

    fun getDown(): Command {
        return setPosition(HoodPositions.Down.angle)
    }

    fun getToOuttake(): Command {
        return setPosition(HoodPositions.TAKEOUT.angle)
    }

    fun getToVertical(): Command {
        return setPosition(HoodPositions.VERTICAL.angle)
    }

    fun moveByController(angle: Angle) : Command{

        return Commands.run({
            val currentAngle = hoodMotor.inputs.position
            positionRequest.withPosition(angle + currentAngle)
            hoodMotor.setControl(positionRequest)
        })

    }

    override fun periodic() {
        hoodMotor.updateInputs()
        Logger.processInputs(name, hoodMotor.inputs)
    }
}
