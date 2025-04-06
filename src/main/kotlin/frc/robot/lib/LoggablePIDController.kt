package frc.robot.lib

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.struct.StructSerializable

class LoggablePIDController(kp: Double, ki: Double, kd: Double) :
    PIDController(kp, ki, kd), StructSerializable {
    val struct: PIDControllerStruct = PIDControllerStruct()
}
