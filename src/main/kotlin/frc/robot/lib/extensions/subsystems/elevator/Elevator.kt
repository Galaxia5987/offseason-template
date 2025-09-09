package frc.robot.lib.extensions.subsystems.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Elevator: SubsystemBase() {
  private  val motor = UniversalTalonFX(
      0,
      config =
          TalonFXConfiguration().apply {
              MotorOutputConfigs().apply {
                  NeutralMode = NeutralModeValue.Brake
                  Inverted = InvertedValue.Clockwise_Positive
              }
              CurrentLimits = CurrentLimitsConfigs().apply {
                  SupplyCurrentLimitEnable = true
                  SupplyCurrentLimit = 70.0
                  StatorCurrentLimitEnable = true
                  StatorCurrentLimit = 70.0
                  SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0
                  SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                  SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0
                  SoftwareLimitSwitch.ReverseSoftLimitEnable = true
              }
              Slot0Configs().apply {
                  Slot0.kP = 1.0
                  Slot0.kI = 0.0
                  Slot0.kD = 0.25
              }
          },
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = SPORCKET_DIAMETERS
      )

    val postionvoltage= PositionVoltage(0.0)
    fun setPosition(position: Distance): Command{
      return Commands.runOnce( {motor.setControl(postionvoltage.withPosition(position.toAngle(SPORCKET_DIAMETERS, GEAR_RATIO)))})
    }
    val voltageRequest= VoltageOut(0.0)
    fun setVoltage(voltage: Voltage): Command {
       return Commands.runOnce( {motor.setControl(voltageRequest.withOutput(voltage))})
    }

    fun GoToL1 (): Command{
        return setPosition(Corallevels.LEVEL1.position)
    }
    fun GoToL2 (): Command{
       return setPosition(Corallevels.LEVEL2.position)
    }
    fun GoToL3 (): Command{
       return setPosition(Corallevels.LEVEL3.position)
    }
    fun GoToL4 (): Command{
        return setPosition(Corallevels.LEVEL4.position)
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs(name,motor.inputs)
    }
}