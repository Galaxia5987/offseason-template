package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.extensions.*

const val ratio = 2.0 / 3.0

val voltageInTake = (-2.5).volts
val voltageOutTake = 2.5.volts
val voltageStop = 0.0.volts

val motorConfig =
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
            FeedbackConfigs().apply { SensorToMechanismRatio = 2.0 / 3.0 }
    }

const val DISTANCE_SENSOR_ID: Int = 0
val DISTANCE_THRESHOLD = 50.mm
val distanceSensorConfig =
    CANrangeConfiguration().apply {
        ProximityParams =
            ProximityParamsConfigs().apply {
                ProximityThreshold = DISTANCE_THRESHOLD[m]
            }
    }
