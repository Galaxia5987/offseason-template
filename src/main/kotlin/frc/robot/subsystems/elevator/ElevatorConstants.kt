package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.extensions.cm

val ratio = 2.0 / 7.0
val wheelDiameter = 3.0.cm

val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply{
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply{
                StatorCurrentLimit = 70.0
                StatorCurrentLimitEnable = true
                SupplyCurrentLimit = 35.0
                SupplyCurrentLimitEnable = true
            }
        Feedback =
            FeedbackConfigs().apply{
                SensorToMechanismRatio = 2.0 / 7.0
            }
        Slot0 =
            Slot0Configs().apply{
                kP = 3.2
                kD = 0.4
            }
    }