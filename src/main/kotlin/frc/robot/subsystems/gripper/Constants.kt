package frc.robot.subsystems.gripper

import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts

const val SENSOR_PORT = 0
val inTakeAndOuttakeL3L4Voltage = 5.0.volts
val outTakeVoltageL1L2 = -5.0.volts
const val GEAR_RATIO = 1.05
val intakeCurrentLimit = 6.0.amps