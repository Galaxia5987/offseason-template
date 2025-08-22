package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Wrist: SubsystemBase() {

    val motor= UniversalTalonFX(
        0,
        config = TalonFXConfiguration().apply {
            MotorOutput= MotorOutputConfigs().apply {
                NeutralMode= NeutralModeValue.Brake
                Inverted= InvertedValue.CounterClockwise_Positive
            }
            CurrentLimits= CurrentLimitsConfigs().apply {
                SupplyCurrentLimitEnable=true
                SupplyCurrentLimit=50.0
                StatorCurrentLimitEnable=true
                StatorCurrentLimit=50.0
            }
        }
    )


    val positionRequest= PositionVoltage(0.0)
    fun setPosition(angle: Angle): Command{
        return Commands.run({motor.setControl(positionRequest.withPosition(angle))})
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(name, motor.inputs)
    }

}