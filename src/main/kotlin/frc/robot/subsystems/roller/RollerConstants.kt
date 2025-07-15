package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.volts

const val MOTOR_ID = 0

val INTAKE_VOLTAGE: Voltage = 3.0.volts
val OUTTAKE_VOLTAGE: Voltage = -INTAKE_VOLTAGE

private val STATOR_CURRENT_LIMIT = 30.amps
private val SUPPLY_CURRENT_LIMIT = STATOR_CURRENT_LIMIT.times(2.0)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
    }
