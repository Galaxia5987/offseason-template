package frc.robot.lib.controllers.structs

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeDouble
import java.nio.ByteBuffer

class ProfiledPIDControllerStruct : Struct<ProfiledPIDController> {
    override fun getTypeClass(): Class<ProfiledPIDController> =
        ProfiledPIDController::class.java

    override fun getTypeName(): String = "ProfiledPIDController"

    override fun getSize(): Int = kSizeDouble * 16 + kSizeBool * 2

    override fun getSchema(): String =
        "double kp;double ki;double kd;double maxVelocity;double maxAcceleration;double iZone;double period;double positionTolerance;double velocityTolerance;double accumulatedError;double goalPosition;double goalVelocity;double setpointPosition;double setpointVelocity;double positionError;double velocityError;bool atSetpoint;bool atGoal;"

    override fun unpack(bb: ByteBuffer?): ProfiledPIDController? = null

    override fun pack(bb: ByteBuffer, value: ProfiledPIDController) {
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
