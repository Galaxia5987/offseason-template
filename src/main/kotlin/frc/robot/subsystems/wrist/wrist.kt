package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.volts

class wrist: SubsystemBase() {

    val motor= TalonFX(1)
    val motorConfigs= TalonFXConfiguration()
    init {
        motorConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
        motorConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        motorConfigs.withCurrentLimits(CurrentLimitsConfigs().withStatorCurrentLimit(12.0)
            .withSupplyCurrentLimitEnable(true )
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimit(12.0))
        motor.configurator.apply(motorConfigs)
    }

    val positionRequest= PositionVoltage(0.0)
    fun setPosition(angle: Angle): Command{
        return Commands.run({motor.setControl(positionRequest.withPosition(angle))})
    }

}