package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.extensions.cm

val ratio = 1.0 / 5.0
val wheelDiameter = 2.0.cm

public val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply{
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply{
                StatorCurrentLimit = 80.0
                StatorCurrentLimitEnable = true
                SupplyCurrentLimit = 40.0
                SupplyCurrentLimitEnable = true
            }
        Feedback =
            FeedbackConfigs().apply{
                SensorToMechanismRatio = 1.0 / 5.0
            }
        Slot0 =
            Slot0Configs().apply{
                kP = 0.0
                kD = 0.0
            }
    }