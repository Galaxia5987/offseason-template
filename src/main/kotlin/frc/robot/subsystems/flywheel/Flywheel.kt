package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degreesPerSecond
import kotlin.math.abs

class Flywheel : SubsystemBase() {
    val motor = TalonFX(motorPort)
    val voltageRequest = VoltageOut(0.0)
    fun setVoltage(VoltageOut: Voltage): Command {
        return Commands.run({ motor.setControl(voltageRequest.withOutput(VoltageOut)) })
    }

    val requestVelocity= VelocityVoltage(0.0)
    fun setVelocity(velocity: AngularVelocity): Command {
        return Commands.runOnce({motor.setControl(requestVelocity.withVelocity(velocity))})
    }
    fun getVelocity(): AngularVelocity{
        return motor.velocity.value;
    }

    fun rampToVelocity(velocity: AngularVelocity) : Command {
        return setVelocity(velocity).andThen(
        Commands.waitUntil { (getVelocity() - velocity) < 10.degreesPerSecond &&
                (getVelocity() - velocity) > -10.degreesPerSecond}
        )
    }

}