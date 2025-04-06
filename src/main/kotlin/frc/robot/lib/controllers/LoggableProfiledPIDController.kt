package frc.robot.lib.controllers

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.lib.controllers.structs.ProfiledPIDControllerStruct

class LoggableProfiledPIDController(kp: Double, ki: Double, kd: Double, constraints: TrapezoidProfile.Constraints?) :
    ProfiledPIDController(kp, ki, kd, constraints), StructSerializable {
    val struct: ProfiledPIDControllerStruct = ProfiledPIDControllerStruct()
}