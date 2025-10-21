package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rot_ps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Shooter : SubsystemBase() {
    val config = TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Slot0 = Slot0Configs().apply {
            kP = 0.0
            kI = 0.0
            kD = 0.0
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            SupplyCurrentLimit = 10.0
            StatorCurrentLimit = 20.0
        }
        Feedback= FeedbackConfigs().apply {

        }
    }
    private val motorShooter: UniversalTalonFX = UniversalTalonFX(0, config = config)
    private val velocityReq: VelocityVoltage = VelocityVoltage(0.0)
    private var setVelocity: AngularVelocity = 0.0.rot_ps

    fun setVelocity(velocity: AngularVelocity) {
        setVelocity = velocity
        motorShooter.setControl(velocityReq.withVelocity(velocity))
    }

    override fun periodic() {
        motorShooter.updateInputs()
        Logger.processInputs("Shooter", motorShooter.inputs)
        Logger.recordOutput("Shooter/setVelocity", setVelocity)
    }
}