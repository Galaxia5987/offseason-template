package frc.robot.subsystems.roller

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Roller : SubsystemBase() {

    private val motor: UniversalTalonFX = UniversalTalonFX(
        MOTOR_ID,
        "",
        MOTOR_CONFIG
    )

    private val sensor = AnalogInput(0)

    private val voltageRequest = VoltageOut(0.0)

    private fun setVoltage(voltage: Voltage): Command =
        runOnce {
            motor.setControl(voltageRequest.withOutput(voltage))
        }.withName("Roller/setVoltage")

    fun intake(): Command = setVoltage(INTAKE_VOLTAGE).withName("Roller/intake")

    fun outtake(): Command = setVoltage(OUTTAKE_VOLTAGE).withName("Roller/outtake")

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Roller", motor.inputs)
    }
}