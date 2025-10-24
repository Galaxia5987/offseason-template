package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.volts

val ratio = 2.0 / 3.0

 val voltageInTake = -2.5.volts
 val voltageOutTake = 2.5.volts
 val voltageStop = 0.0.volts

public val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimit = 50.0
                StatorCurrentLimitEnable = true
                SupplyCurrentLimit = 25.0
                SupplyCurrentLimitEnable = true
            }
        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = 2.0 / 3.0
            }
    }

