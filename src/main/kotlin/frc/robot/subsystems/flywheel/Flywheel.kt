package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Flywheel : SubsystemBase(){
    val motor= TalonFX(motorPort)
    val voltageRequest= VoltageOut(0.0)
    fun setVoltage(VoltageOut: Voltage): Command {
     return Commands.run({motor.setControl(voltageRequest.withOutput(VoltageOut))})
    }
    val positionRequest= PositionVoltage(0.0)
    fun setAngle(angle: Angle){
        motor.setControl(positionRequest.withPosition(angle))
    }

}