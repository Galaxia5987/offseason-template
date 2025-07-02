package frc.robot.subsystems

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.kilogramSquareMeters
import frc.robot.lib.universal_motor.UniversalMotor

class Roller : SubsystemBase() {
    private val motor = UniversalMotor(
        port = 0,
        momentOfInertia = (0.003).kilogramSquareMeters,
    )
    private val voltageRequest = VoltageOut(0.0)

    fun setVoltage(voltage: Voltage): Command = this.runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    override fun periodic() {

    }
}