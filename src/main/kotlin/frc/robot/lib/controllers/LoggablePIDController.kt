package frc.robot.lib.controllers

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.lib.controllers.structs.PIDControllerStruct

class LoggablePIDController(kp: Double, ki: Double, kd: Double) :
    PIDController(kp, ki, kd), StructSerializable {
    companion object {
        val struct: PIDControllerStruct = PIDControllerStruct()
    }
}
