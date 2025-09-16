package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
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
    val motor = UniversalTalonFX(16, config = motorConfig, gearRatio = )



    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(name, motor.inputs)
    }
}