package frc.robot.lib

import com.ctre.phoenix6.configs.Slot0Configs

/**
 * An extension function to convert a [Gains] type to the [Slot0Configs] that
 * the motor uses.
 */
fun Gains.toSlotConfig() =
    Slot0Configs().apply {
        kP = this@toSlotConfig.kP
        kI = this@toSlotConfig.kI
        kD = this@toSlotConfig.kD
        kA = this@toSlotConfig.kA
        kS = this@toSlotConfig.kS
        kV = this@toSlotConfig.kV
        kG = this@toSlotConfig.kG
    }

data class Gains(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val kS: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kG: Double = 0.0
)
