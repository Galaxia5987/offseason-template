package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.LoggedNetworkGains
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team5987.annotation.LoggedOutput

private const val LOGGING_PREFIX = "AutoAlignment"

private val xGains = LoggedNetworkGains("x Gains", 4.0)
private val yGains =
    LoggedNetworkGains(
        "y Gains",
    )

private val thetaGains = LoggedNetworkGains("ϴ Gains", 6.0)
private val linearMaxVelocity =
    LoggedNetworkNumber("$LOGGING_PREFIX/linearMaxVelocity", 4.69)
private val linearMaxAcceleration =
    LoggedNetworkNumber("$LOGGING_PREFIX/linearMaxAcceleration", 2.8)

private var rotationalMaxVelocity =
    LoggedNetworkNumber("$LOGGING_PREFIX/rotationMaxVelocity", 7.0)
private var rotationalMaxAcceleration =
    LoggedNetworkNumber("$LOGGING_PREFIX/rotationMaxAcceleration", 360.0)

private val linearLimits
    get() = Constraints(linearMaxVelocity.get(), linearMaxAcceleration.get())

private val rotationalLimits
    get() =
        Constraints(
            rotationalMaxVelocity.get(),
            rotationalMaxAcceleration.get()
        )

@LoggedOutput("X controller", LOGGING_PREFIX)
var xController =
    ProfiledPIDController(
        xGains.kP.get(),
        xGains.kI.get(),
        xGains.kD.get(),
        linearLimits
    )

@LoggedOutput("Y controller", LOGGING_PREFIX)
var yController =
    ProfiledPIDController(
        yGains.kP.get(),
        yGains.kI.get(),
        yGains.kD.get(),
        linearLimits
    )

@LoggedOutput("ϴ controller", LOGGING_PREFIX)
var thetaController =
    ProfiledPIDController(
            thetaGains.kP.get(),
            thetaGains.kI.get(),
            thetaGains.kD.get(),
            rotationalLimits
        )
        .apply { enableContinuousInput(-Math.PI, Math.PI) }

@LoggedOutput(path = LOGGING_PREFIX)
var atGoal: Trigger =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atGoal)

fun updateProfiledPIDGains() {
    mapOf(
            xController to xGains,
            yController to yGains,
            thetaController to thetaGains
        )
        .forEach { (controller, gains) ->
            controller.setPID(gains.kP.get(), gains.kI.get(), gains.kD.get())
        }
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPIDGains()
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisSpeeds) {
    xController.reset(botPose.x, botSpeeds.vxMetersPerSecond)
    yController.reset(botPose.y, botSpeeds.vyMetersPerSecond)
    thetaController.reset(
        botPose.rotation.radians,
        botSpeeds.omegaRadiansPerSecond
    )
}

fun setTolerance(pose2d: Pose2d) {
    xController.setTolerance(pose2d.x)
    yController.setTolerance(pose2d.y)
    thetaController.setTolerance(pose2d.rotation.radians)
}

/**
 * Returns field relative chassis speeds to the selected goal.
 * @botPose the current pose of the robot
 */
fun getSpeedSetpoint(botPose: Pose2d): () -> ChassisSpeeds = {
    ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians)
    )
}
