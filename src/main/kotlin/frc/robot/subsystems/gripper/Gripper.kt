package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.centimeters
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Gripper : SubsystemBase() {
    var sensorDistance: Distance = Units.Meters.zero()
    val hasCoral = Trigger{sensorDistance <= coralInTheGripperConstant}
    private val sensor = AnalogInput(SENSOR_PORT)
    private val distanceFilter = MedianFilter(3)
    private val motor =
        UniversalTalonFX(
            16,
            gearRatio = frc.robot.subsystems.gripper.GEAR_RATIO,
            config =
                TalonFXConfiguration().apply {
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.CounterClockwise_Positive
                    }
                    CurrentLimits =
                        CurrentLimitsConfigs().apply {
                            SupplyCurrentLimitEnable = true
                            SupplyCurrentLimit = 70.0
                            StatorCurrentLimitEnable = true
                            StatorCurrentLimit = 70.0
                        }
                }
        )
    val voltageRequest = VoltageOut(0.0)

    fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({
            motor.setControl(voltageRequest.withOutput(voltage))
        })
    }

    fun output(): Command {
        return setVoltage(outTakeVoltage)
    }

    fun input(): Command {
        return setVoltage(inTakeVoltage)
    }
    fun stop(): Command {
        return setVoltage(0.0.volts)
    }

    fun intakeByGripperSensor() : Command {
        return input().andThen(Commands.waitUntil(hasCoral).andThen(stopIntakeOuttake()))
    }

    fun outtakeByGripperSensor() : Command{
        return output().andThen(Commands.waitUntil(hasCoral.negate()).andThen(stopIntakeOuttake()))
    }

    fun stopIntakeOuttake() : Command{
        return setVoltage(0.0.volts)
    }

    override fun periodic() {
        var calculatedDistance =
            distanceFilter.calculate(4800 / (200 * sensor.voltage - 20.0))
        if (calculatedDistance < 0) {
            calculatedDistance = 80.0
        }
        sensorDistance = calculatedDistance.centimeters

        Logger.recordOutput("sensorDistance", sensorDistance)
    }
}
