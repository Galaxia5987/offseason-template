package frc.robot.subsystems.roller

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.StartEndCommand
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Roller : SubsystemBase() {

    private val motor: UniversalTalonFX =
        UniversalTalonFX(MOTOR_ID, "", MOTOR_CONFIG, 0.3.kg2m)

    private val voltageRequest = VoltageOut(0.0)

    private fun setVoltage(voltage: Voltage): Command =
        runOnce { motor.setControl(voltageRequest.withOutput(voltage)) }
            .withName("Roller/setVoltage")

    fun intake(): Command =
        this.setVoltage(INTAKE_VOLTAGE).withName("Roller/intake")

    fun stop(): Command = setVoltage(0.0.volts).withName("Roller/stop")

    fun outtake(): Command =
        this.setVoltage(OUTTAKE_VOLTAGE).withName("Roller/outtake")

    fun handleIntake(): Command = StartEndCommand(intake(), stop())

    //    fun handleIntake(): Command = PrintCommand("ASdasasdasda")
    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Roller", motor.inputs)
    }
}
