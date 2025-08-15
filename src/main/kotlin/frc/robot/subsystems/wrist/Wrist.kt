package frc.robot.subsystems.wrist

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees

class Wrist : SubsystemBase() {
    val wristMotor = TalonFX(wristPort)
    val positionRequest = PositionVoltage(0.0.degrees)
    val wristMotorConfig = TalonFXConfiguration()

     init {
         wristMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
         wristMotorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
         wristMotorConfig.CurrentLimits.withStatorCurrentLimit(12.0).withSupplyCurrentLimit(12.0).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true)
     }

    fun setPosition(position: Angle): Command{
        positionRequest.withPosition(position)
        return Commands.run({wristMotor.setControl(positionRequest)})
    }
}