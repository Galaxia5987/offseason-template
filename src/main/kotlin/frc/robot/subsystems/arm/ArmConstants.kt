package frc.robot.subsystems.arm

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.toSlotConfig

const val MOTOR_ID = 1

enum class ArmPosition(val angle: Angle) {
    INTAKING(0.deg),
    UP(90.deg)
}

private val STATOR_CURRENT_LIMIT = 30.amps
private val SUPPLY_CURRENT_LIMIT = STATOR_CURRENT_LIMIT.times(2.0)

private val pidConfig = Gains(kP = 0.4, kD = 0.2)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = pidConfig.toSlotConfig()

        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
    }
