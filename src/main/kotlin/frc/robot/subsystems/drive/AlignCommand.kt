package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import frc.robot.drive
import frc.robot.lib.LoggedHolonomicDriveController

private val translationController = PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD)

private val angularController = ProfiledPIDController(
    ANGULAR_KP,
    ANGULAR_KI,
    ANGULAR_KD,
    ANGLE_CONSTRAINTS
)

val controller =
    LoggedHolonomicDriveController(
        translationController,
        angularController
    )
        .apply { setTolerance(TOLERANCE) }

fun alignToPose(
    goalPose: Pose2d,
    linearVelocity: LinearVelocity = MetersPerSecond.zero(),
    tolerance: Pose2d = TOLERANCE,
): Command =
        drive
            .defer {
                run({
                    drive.runVelocity(
                        controller.apply {
                            setTolerance(tolerance)
                        }.calculate(
                            drive.pose,
                            goalPose,
                            linearVelocity.`in`(MetersPerSecond),
                            goalPose.rotation
                        )
                    )
                })

            }
        .until { controller.atReference() }
        .withName("Drive/AlignToPose")
