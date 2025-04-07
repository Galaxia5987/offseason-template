package frc.robot.lib.controllers.structs

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeDouble
import frc.robot.lib.controllers.LoggableProfiledPIDController
import java.nio.ByteBuffer

class ProfiledPIDControllerStruct : Struct<LoggableProfiledPIDController> {
    override fun getTypeClass(): Class<LoggableProfiledPIDController> =
        LoggableProfiledPIDController::class.java

    override fun getTypeName(): String = "ProfiledPIDController"

    override fun getSize(): Int = kSizeDouble * 16 + kSizeBool * 2

    override fun getSchema(): String =
        "double kp;double ki;double kd;double maxVelocity;double maxAcceleration;double iZone;double period;double positionTolerance;double velocityTolerance;double accumulatedError;double goalPosition;double goalVelocity;double setpointPosition;double setpointVelocity;double positionError;double velocityError;bool atSetpoint;bool atGoal;"

    override fun unpack(bb: ByteBuffer): LoggableProfiledPIDController {
        val kp = bb.getDouble()
        val ki = bb.getDouble()
        val kd = bb.getDouble()
        val maxVelocity = bb.getDouble()
        val maxAcceleration = bb.getDouble()

        // Skip the following fields:
        // iZone, period, positionTolerance, velocityTolerance, accumulatedError, goalPosition,
        // goalVelocity, setpointPosition, setpointVelocity, positionError, velocityError,
        // atSetpoint, atGoal
        bb.position(bb.position() + kSizeBool * 11 + kSizeBool * 2)

        return LoggableProfiledPIDController(
            kp,
            ki,
            kd,
            TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
        )
    }

    override fun pack(bb: ByteBuffer, value: LoggableProfiledPIDController) {
        listOf(
                value.p,
                value.i,
                value.d,
                value.constraints.maxVelocity,
                value.constraints.maxAcceleration,
                value.iZone,
                value.period,
                value.positionTolerance,
                value.velocityTolerance,
                value.accumulatedError,
                value.goal.position,
                value.goal.velocity,
                value.setpoint.position,
                value.setpoint.velocity,
                value.positionError,
                value.velocityError,
            )
            .forEach { bb.putDouble(it) }

        bb.put(if (value.atSetpoint()) 1 else 0)
        bb.put(if (value.atGoal()) 1 else 0)
    }
}
