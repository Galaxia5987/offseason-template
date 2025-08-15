package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.volts

class FlyWheel : SubsystemBase(){
    val flywheelMotor = TalonFX(FlywheelPort)
    val voltageRequest = VoltageOut(0.0.volts)
    val angleRequest = PositionVoltage(0.0.degrees)

    fun setVoltage(voltage: Voltage): Command {
        voltageRequest.withOutput(voltage)
        return Commands.run({flywheelMotor.setControl(voltageRequest)})
    }

    fun setAngle(angle: Angle): Command{
        angleRequest.withPosition(angle)
        return Commands.run({flywheelMotor.setControl(angleRequest)})
    }
}