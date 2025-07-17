package frc.robot.lib.extensions

import com.ctre.phoenix6.configs.Slot0Configs
import frc.robot.lib.Gains

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
