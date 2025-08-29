package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Flywheel : SubsystemBase() {
    val motor =
        UniversalTalonFX(
            motorPort,
            config =
                TalonFXConfiguration().apply {
                    MotorOutput =
                        MotorOutputConfigs().apply {
                            NeutralMode = NeutralModeValue.Brake
                            Inverted = InvertedValue.Clockwise_Positive
                        }
                    CurrentLimits =
                        CurrentLimitsConfigs().apply {
                            SupplyCurrentLimitEnable = true
                            SupplyCurrentLimit = 100.0
                            StatorCurrentLimitEnable = true
                            SupplyCurrentLimit = 100.0
                        }
                    Slot0.kP=1.0
                    Slot0.kI=0.0
                    Slot0.kD=0.0
                }

        )

    val voltageRequest = VoltageOut(0.0)
    fun setVoltage(VoltageOut: Voltage): Command {
        return Commands.runOnce({
            motor.setControl(voltageRequest.withOutput(VoltageOut))
        })
    }

    val requestVelocity = VelocityVoltage(0.0)
    fun setVelocity(velocity: AngularVelocity): Command {
        return Commands.runOnce({
            motor.setControl(requestVelocity.withVelocity(velocity))
        })
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(name, motor.inputs)
    }
}
