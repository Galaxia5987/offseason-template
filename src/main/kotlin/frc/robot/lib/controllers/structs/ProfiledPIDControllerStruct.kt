package frc.robot.lib.controllers.structs

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeDouble
import frc.robot.lib.controllers.LoggableProfiledPIDController
import java.nio.ByteBuffer

class ProfiledPIDControllerStruct: Struct<LoggableProfiledPIDController> {
    override fun getTypeClass(): Class<LoggableProfiledPIDController> = LoggableProfiledPIDController::class.java

    override fun getTypeName(): String = "ProfiledPIDController"

    override fun getSize(): Int = kSizeDouble * 16 + kSizeBool * 2

    override fun getSchema(): String =
        "double kp;double ki;double kd;double maxVelocity;double maxAcceleration;double iZone;double period;double positionTolerance;double velocityTolerance;double accumulatedError;double goalPosition;double goalVelocity;bool atGoal;double setpointPosition;double setpointVelocity;bool atSetpoint;double positionError;double velocityError;"

    override fun unpack(bb: ByteBuffer): LoggableProfiledPIDController {
        val kp = bb.getDouble()
        val ki = bb.getDouble()
        val kd = bb.getDouble()
        val maxVelocity = bb.getDouble()
        val maxAcceleration = bb.getDouble()

        // Move the buffer forward
        bb.position(bb.position() + kSizeBool * 11 + kSizeBool * 2)

        return LoggableProfiledPIDController(kp, ki, kd, TrapezoidProfile.Constraints(maxVelocity, maxAcceleration))
    }

    override fun pack(bb: ByteBuffer, value: LoggableProfiledPIDController) {
        bb.putDouble(value.p)
        bb.putDouble(value.i)
        bb.putDouble(value.d)
        bb.putDouble(value.constraints.maxVelocity)
        bb.putDouble(value.constraints.maxAcceleration)
        bb.putDouble(value.iZone)
        bb.putDouble(value.period)
        bb.putDouble(value.positionTolerance)
        bb.putDouble(value.velocityTolerance)
        bb.putDouble(value.accumulatedError)
        bb.putDouble(value.goal.position)
        bb.putDouble(value.goal.velocity)
        bb.put(if (value.atGoal()) 1 else 0)
        bb.putDouble(value.setpoint.position)
        bb.putDouble(value.setpoint.velocity)
        bb.put(if (value.atGoal()) 1 else 0)
        bb.putDouble(value.positionError)
        bb.putDouble(value.velocityError)
    }
}