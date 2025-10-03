package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Elevator : SubsystemBase() {
    private val motorConfigs =
        TalonFXConfiguration().apply {
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }

            CurrentLimits =
                CurrentLimitsConfigs().apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = 70.0
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = 70.0
                    SoftwareLimitSwitch.ForwardSoftLimitThreshold = 27.5
                    SoftwareLimitSwitch.ForwardSoftLimitEnable = false
                    SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
                    SoftwareLimitSwitch.ReverseSoftLimitEnable = false
                }

            Slot0Configs().apply {
                Slot0.kP = 1.0
                Slot0.kI = 0.0
                Slot0.kD = 0.25
            }
        }

    private val mainMotor =
        UniversalTalonFX(
            port = 13,
            config = motorConfigs,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = SPORCKET_DIAMETERS
        )

    private val auxMotor =
        UniversalTalonFX(
            14,
            config = motorConfigs,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = SPORCKET_DIAMETERS
        )

    val followerRequest = Follower(13, false)

    var setPoint = 0.m
    val positionVoltage = PositionVoltage(0.0)
    fun setPosition(position: Distance): Command {
        return Commands.runOnce({
            setPoint = position
            auxMotor.setControl(
                positionVoltage.withPosition(
                    position.toAngle(SPORCKET_DIAMETERS, GEAR_RATIO)
                )
            )
        })
    }

    val voltageRequest = VoltageOut(0.0)
    fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({
            auxMotor.setControl(voltageRequest.withOutput(voltage))
        })
    }

    fun GoToL0(): Command {
        return setPosition(Corallevels.LEVEL0.position)
    }

    fun GoToL1(): Command {
        return setPosition(Corallevels.LEVEL1.position)
    }

    fun GoToL2(): Command {
        return setPosition(Corallevels.LEVEL2.position)
    }

    fun GoToL3(): Command {
        return setPosition(Corallevels.LEVEL3.position)
    }

    fun GoToL4(): Command {
          return setPosition(Corallevels.LEVEL4.position)
        }

    init {
        auxMotor.setControl(followerRequest)
    }
    override fun periodic() {
        auxMotor.updateInputs()
        Logger.processInputs(name, mainMotor.inputs)
        Logger.recordOutput("levels", setPoint)
    }
}
