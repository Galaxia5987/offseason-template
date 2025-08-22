package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
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
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.flywheel.motorPort
import org.littletonrobotics.junction.Logger


class Hood : SubsystemBase() {
    val motor = UniversalTalonFX(
        motorPort,
        config = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = 10.0
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = 10.0
            }
        }
    )

    val voltageRequest = VoltageOut(0.0)
    fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({ motor.setControl(voltageRequest.withOutput(voltage)) })
    }

    val positionRequest = PositionVoltage(0.0)
    fun setPosition(angle: Angle): Command {
        return Commands.runOnce({ motor.setControl(positionRequest.withPosition(angle)) })
    }


    fun closeHood(): Command {
        return setPosition(Hoodangles.ZERO.angle)
    }

    fun steepestAngle(): Command {
        return setPosition(Hoodangles.TWENTY)
    }

    fun setPosition(angle: Hoodangles): Command {
        return setPosition(angle.angle)
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(name, motor.inputs)

    }
}
