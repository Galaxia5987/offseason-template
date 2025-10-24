package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.seconds
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier

class Gripper : SubsystemBase() {
    val hasCoral = Trigger { motor.inputs.current >= intakeCurrentLimit }

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

    fun outtake(isReverse: BooleanSupplier): Command {
        return Commands.either(
            setVoltage(inTakeAndOuttakeL3L4Voltage).andThen(Commands.waitTime(0.2.seconds)), setVoltage(outTakeVoltageL1L2).andThen(Commands.waitTime(0.2.seconds)), isReverse
        )
    }

    fun intake(): Command {
        return setVoltage(inTakeAndOuttakeL3L4Voltage)
    }

    fun stop(): Command {
        return setVoltage(0.0.volts)
    }

    fun intakeByGripperSensor(): Command {
        return intake().andThen(Commands.waitUntil(hasCoral)).andThen(stopIntakeOuttake())
    }

    fun stopIntakeOuttake(): Command{
        return setVoltage(0.0.volts)
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Gripper", motor.inputs)
    }
}
