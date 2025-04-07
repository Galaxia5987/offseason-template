package frc.robot.lib.controllers.structs

import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeDouble
import frc.robot.lib.controllers.LoggablePIDController
import java.nio.ByteBuffer

class PIDControllerStruct : Struct<LoggablePIDController> {

    override fun getTypeClass(): Class<LoggablePIDController> =
        LoggablePIDController::class.java

    override fun getTypeName() = "PIDController"

    override fun getSize(): Int = kSizeDouble * 11 + kSizeBool

    override fun getSchema(): String =
        "double kp;double ki;double kd;double iZone;double period;double errorTolerance;double errorDerivativeTolerance;double accumulatedError;double setpoint;double error;double errorDerivative;bool atSetpoint"

    override fun unpack(bb: ByteBuffer): LoggablePIDController {
        val kp = bb.getDouble()
        val ki = bb.getDouble()
        val kd = bb.getDouble()

        // Skip the following fields:
        // iZone, period, errorTolerance, errorDerivativeTolerance, accumulatedError, setpoint, error, errorDerivative, atSetpoint
        bb.position(bb.position() + kSizeBool + kSizeDouble * 8)

        return LoggablePIDController(kp, ki, kd)
    }

    override fun pack(bb: ByteBuffer, value: LoggablePIDController) {
        listOf(
            value.p,
            value.i,
            value.d,
            value.iZone,
            value.period,
            value.errorTolerance,
            value.errorDerivativeTolerance,
            value.accumulatedError,
            value.setpoint,
            value.error,
            value.errorDerivativeTolerance,
            value.error,
            value.errorDerivativeTolerance
        ).forEach(bb::putDouble)

        bb.put(if (value.atSetpoint()) 1 else 0)
    }
}
