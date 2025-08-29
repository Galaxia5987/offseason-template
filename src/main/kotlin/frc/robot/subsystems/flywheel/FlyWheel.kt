package frc.robot.subsystems.flywheel

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
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class FlyWheel : SubsystemBase() {
    val flywheelMotorConfig = TalonFXConfiguration().apply {
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
    val flywheelMotor = UniversalTalonFX(FlywheelPort, "flywheelMotor", flywheelMotorConfig)
    val voltageRequest = VoltageOut(0.0.volts)


    fun setVoltage(voltage: Voltage): Command {
        voltageRequest.withOutput(voltage)
        return Commands.runOnce({ flywheelMotor.setControl(voltageRequest) })
    }

    override fun periodic() {
        flywheelMotor.updateInputs()
        Logger.processInputs(name, flywheelMotor.inputs)
    }
}
