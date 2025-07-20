package frc.robot.lib

import com.ctre.phoenix6.configs.Slot0Configs

data class Gains(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val kS: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kG: Double = 0.0
) {
    /**
     * A function to convert a [Gains] type to the [Slot0Configs] that the motor
     * uses.
     */
    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@Gains.kP
            kI = this@Gains.kI
            kD = this@Gains.kD
            kA = this@Gains.kA
            kS = this@Gains.kS
            kV = this@Gains.kV
            kG = this@Gains.kG
        }
}
