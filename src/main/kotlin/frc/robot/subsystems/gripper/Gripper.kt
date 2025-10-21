package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Gripper : SubsystemBase() {

    private val motor1: UniversalTalonFX =
        UniversalTalonFX(0, config = config, gearRatio = ratio)
    private val setVoltageRequest: VoltageOut = VoltageOut(0.0)

    fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({
            motor1.setControl(setVoltageRequest.withOutput(voltage))
        })
    }

    fun inTake(): Command { //takeCylinder
        return setVoltage(voltageInTake)
    }

    fun outTake(): Command{ //throwCylinder
        return setVoltage(voltageOutTake)
    }

    fun stop(): Command{ //stops
        return setVoltage(voltageStop)
    }

    override fun periodic() {
        Gripper.motor1.updateInputs()
        Logger.processInputs("Gripper", Gripper.motor1.inputs)
    }
}