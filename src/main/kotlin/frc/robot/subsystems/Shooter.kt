package frc.robot.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rotationsPerSecond
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.elevator.wheelDiameter
import org.littletonrobotics.junction.Logger

object Shooter: SubsystemBase() {
    private val config =
        TalonFXConfiguration().apply {
            MotorOutput =
                MotorOutputConfigs().apply {
                    NeutralMode = NeutralModeValue.Coast
                    Inverted = InvertedValue.CounterClockwise_Positive
                }
            CurrentLimits =
                CurrentLimitsConfigs().apply {
                    StatorCurrentLimit = 60.0
                    StatorCurrentLimitEnable = true
                    SupplyCurrentLimit = 30.0
                    SupplyCurrentLimitEnable = true
                }
            Feedback =
                FeedbackConfigs().apply {
                    SensorToMechanismRatio = 1.0 / 5.0
                    RotorToSensorRatio = 1.5
                }
            Slot0 =
                Slot0Configs().apply {
                    kP = 1.04
                    kD = 0.00000058
                }
        }
    private val motor1: UniversalTalonFX =
        UniversalTalonFX(port = 0, config = config, gearRatio = 1.0 / 5.0, linearSystemWheelDiameter = wheelDiameter)
    private val motor2: UniversalTalonFX =
        UniversalTalonFX(port = 1, config = config, gearRatio = 1.0 / 5.0)
    private val velocityVoltageRequest: VelocityVoltage = VelocityVoltage(0.0)
    private var setVelocity: AngularVelocity = 0.rotationsPerSecond

    init {
        motor2.setControl(Follower(0, true))
    }

    fun setVelocity(velocity: AngularVelocity): Command {
        return Commands.runOnce({
            setVelocity = velocity
            motor1.setControl(velocityVoltageRequest.withVelocity(velocity))})
    }

    override fun periodic() {
        motor1.updateInputs()
        Logger.processInputs("Shooter", motor1.inputs)
        Logger.recordOutput("Shooter/setpoint", setVelocity)
    }
}