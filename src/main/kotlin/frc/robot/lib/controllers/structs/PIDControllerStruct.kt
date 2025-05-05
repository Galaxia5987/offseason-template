package frc.robot.lib.controllers.structs

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.Struct.kSizeBool
import edu.wpi.first.util.struct.Struct.kSizeDouble
import java.nio.ByteBuffer

class PIDControllerStruct : Struct<PIDController> {

    override fun getTypeClass(): Class<PIDController> =
        PIDController::class.java

    override fun getTypeName() = "PIDController"

    override fun getSize(): Int = kSizeDouble * 13 + kSizeBool

    override fun getSchema(): String =
        "double kp;double ki;double kd;double iZone;double period;double errorTolerance;double errorDerivativeTolerance;double accumulatedError;double setpoint;double error;double errorDerivative;bool atSetpoint"

    override fun unpack(bb: ByteBuffer?): PIDController? = null

    override fun pack(bb: ByteBuffer, value: PIDController) {
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
            )
            .forEach(bb::putDouble)

        bb.put(if (value.atSetpoint()) 1 else 0)
    }
}
