package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer.isFinished12
import frc.robot.RobotContainer.isFinished34
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.wrist.GEAR_RATIO
import org.littletonrobotics.junction.Logger


class Gripper : SubsystemBase() {
    val motorConfig =
        TalonFXConfiguration().apply {
            MotorOutput =
                MotorOutputConfigs().apply {
                    NeutralMode = NeutralModeValue.Coast
                    Inverted = InvertedValue.CounterClockwise_Positive
                }
            CurrentLimits =
                CurrentLimitsConfigs().apply {
                    StatorCurrentLimitEnable = true
                    SupplyCurrentLimitEnable = true
                    StatorCurrentLimit = 20.0
                    SupplyCurrentLimit = 40.0
                }
        }
    val motor = UniversalTalonFX(16, config = motorConfig, gearRatio = frc.robot.subsystems.gripper.GEAR_RATIO)
    val intakeVoltageRequest = VoltageOut(12.0.volts)
    val outtakeVoltageRequests = VoltageOut((-12.0).volts)


    fun intakeAndOuttake3And4() : Command {
        return Commands.run({motor.setControl(intakeVoltageRequest)})
    }

    fun outtake1And2() : Command{
        return Commands.run({motor.setControl(outtakeVoltageRequests)})
    }
    fun stop() : Command{
        return Commands.run({
            motor.setControl(VoltageOut(0.0.volts)).andThen({isFinished12 = false}) })
    }

    init {
        motor.setControl(intakeVoltageRequest)
    }
    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(name, motor.inputs)
    }
}