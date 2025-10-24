package frc.robot.subsystems.gripper

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Gripper : SubsystemBase() {

    private val motor1: UniversalTalonFX =
        UniversalTalonFX(0, config = motorConfig, gearRatio = ratio)
    private val distanceSensor =
        UnifiedCANRange(
            DISTANCE_SENSOR_ID,
            configuration = distanceSensorConfig,
            subsystemName = name
        )
    private val hasBall = Trigger { distanceSensor.isInRange }

    private val setVoltageRequest: VoltageOut = VoltageOut(0.0)

    private fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({
            motor1.setControl(setVoltageRequest.withOutput(voltage))
        })
    }

    fun inTake(): Command { // takeCylinder
        return setVoltage(voltageInTake)
    }

    fun outTake(): Command { // throwCylinder
        return setVoltage(voltageOutTake)
    }

    fun stop(): Command { // stops
        return setVoltage(voltageStop)
    }

    fun inTakeUntilSenses(): Command {
        return Commands.sequence(inTake(), Commands.waitUntil(hasBall), stop())
    }

    override fun periodic() {
        motor1.updateInputs()
        distanceSensor.updateInputs()
        Logger.processInputs(name, motor1.inputs)
    }
}
