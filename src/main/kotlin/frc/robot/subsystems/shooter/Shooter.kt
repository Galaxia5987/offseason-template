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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rot_ps
import frc.robot.lib.extensions.seconds
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Shooter : SubsystemBase() {
    val config = TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Slot0 = Slot0Configs().apply {
            kP = 0.0
            kD = 0.0
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            SupplyCurrentLimit = 10.0
            StatorCurrentLimit = 20.0
        }
        Feedback = FeedbackConfigs().apply {

        }
    }
    private val motorShooter: UniversalTalonFX = UniversalTalonFX(0, config = config)
    private val velocityReq: VelocityVoltage = VelocityVoltage(0.0)
    private var setpointVelocity: AngularVelocity = 0.0.rot_ps

    fun setVelocity(velocity: AngularVelocity) {
        setpointVelocity = velocity
        motorShooter.setControl(velocityReq.withVelocity(velocity))
    }

    override fun periodic() {
        motorShooter.updateInputs()
        Logger.processInputs("Shooter", motorShooter.inputs)
        Logger.recordOutput("Shooter/setVelocity", setpointVelocity)
    }

    fun Shoot(): Command {
        return Commands.runOnce({
            setVelocity(3.0.rot_ps)
        })
    }

    fun Stop(): Command {
        return Commands.runOnce({
            setVelocity(0.0.rot_ps)
        })
    }

    fun ShootAndStop(): Command {
        return Shoot().andThen(Commands.waitTime(3.0.seconds)).andThen(Stop()).withName("ShootAndStop")
            .andThen(PrintHello())
    }

    fun PrintHello(): Command {
        return Commands.run({ println("Hello!") }).until(::isAtSetVelocity)
    }

    fun isAtSetVelocity(): Boolean {
        return (motorShooter.inputs.velocity - setpointVelocity) < 0.1.rot_ps &&
                (motorShooter.inputs.velocity - setpointVelocity) > (-0.1).rot_ps
    }
}


class DoStuffCommand : Command {
    constructor()

    val timer = edu.wpi.first.wpilibj.Timer()
    override fun initialize() {
        Shooter.setVelocity(5.0.rot_ps)
        timer.start()
    }

    override fun execute() {
        println("Moving")
    }

    override fun end(interrupted: Boolean) {
        Shooter.setVelocity(0.0.rot_ps)
    }

    override fun isFinished(): Boolean {
        return timer.get() > 3.0
    }

}