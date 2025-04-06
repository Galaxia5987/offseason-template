package frc.robot.lib

import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeDouble
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.nio.ByteBuffer

class PIDControllerStruct : Struct<LoggablePIDController> {
    override fun getTypeClass(): Class<LoggablePIDController> =
        LoggablePIDController::class.java

    override fun getTypeName(): String {
        return "PIDController"
    }

    override fun getSize(): Int = kSizeDouble * 11 + kSizeBool

    override fun getSchema(): String =
        "double kp;double ki;double kd;double iZone;double period;double errorTolerance;double errorDerivativeTolerance;double accumulatedError;double setpoint;bool atSetpoint;double error;double errorDerivative"

    override fun unpack(bb: ByteBuffer): LoggablePIDController {
        val kp = bb.getDouble()
        val ki = bb.getDouble()
        val kd = bb.getDouble()

        val iZone = bb.getDouble()
        val period = bb.getDouble()
        val errorTolerance = bb.getDouble()
        val errorDerivativeTolerance = bb.getDouble()
        val accumulatedError = bb.getDouble()
        val setpoint = bb.getDouble()
        val atSetpoint = bb.get()
        val error = bb.getDouble()
        val errorDerivative = bb.getDouble()
        return LoggablePIDController(kp, ki, kd)
    }

    override fun pack(bb: ByteBuffer, value: LoggablePIDController) {
        bb.putDouble(value.p)
        bb.putDouble(value.i)
        bb.putDouble(value.d)

        bb.putDouble(value.iZone)
        bb.putDouble(value.period)
        bb.putDouble(value.errorTolerance)
        bb.putDouble(value.errorDerivativeTolerance)
        bb.putDouble(value.accumulatedError)
        bb.putDouble(value.setpoint)
        bb.put(if (value.atSetpoint()) 1 else 0)
        bb.putDouble(value.error)
        bb.putDouble(value.errorDerivativeTolerance)
    }
}
