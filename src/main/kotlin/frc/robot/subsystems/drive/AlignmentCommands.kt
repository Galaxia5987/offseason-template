package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.controllers.ALIGNMENT_LOGGING_PATH
import frc.robot.lib.controllers.TunableHolonomicDriveController
import frc.robot.lib.controllers.TunableProfiledDriveController
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.sec
import org.littletonrobotics.junction.Logger

private val translationController =
    PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD)

private val angularController =
    ProfiledPIDController(
        ANGULAR_KP,
        ANGULAR_KI,
        ANGULAR_KD,
        ANGULAR_CONSTRAINTS
    )

val controller =
    TunableHolonomicDriveController(translationController, angularController)
        .apply { setTolerance(TOLERANCE) }

val profiledController = TunableProfiledDriveController(
    PROFILED_TRANSLATION_GAINS,
    PROFILED_TRANSLATION_CONSTRAINTS,
    TOLERANCE,
    PROFILED_ROTATION_GAINS,
    ANGULAR_CONSTRAINTS,
    { drive.pose },
    { drive.chassisSpeeds }
)

/**
 * Creates a command that aligns the robot to a given goal pose using a
 * holonomic drive controller.
 *
 * A [Command] that aligns the robot to the specified [goalPose].
 *
 * This command will:
 * - Continuously update the robot's chassis speeds using the provided goal pose
 * and desired linear velocity.
 * - Stop when the robot's pose is within the specified tolerance of the goal.
 *
 * This is useful for aligning the robot to a specific point on the field, often
 * before performing an action like shooting or intaking.
 *
 * ### Example Usage:
 * ```kotlin
 * driverController.circle().whileTrue(
 *     alignToPose(
 *         getPose2d(2.0, 2.0, Rotation2d.fromDegrees(90.0))
 *     )
 * )
 * ```
 * This will continuously align the robot to the pose (2.0, 2.0) while the
 * "circle" button is held.
 *
 * @param goalPose The target pose that the robot should align to.
 * @param linearVelocity The desired linear velocity when driving to the pose.
 * Defaults to 0 m/s.
 * @param tolerance The acceptable tolerance around the goal pose to consider
 * alignment complete. Defaults to [TOLERANCE].
 * @param holonomicController The holonomic controller to use for the alignment.
 * Defaults to [controller]
 */
fun alignToPose(
    goalPose: Pose2d,
    linearVelocity: LinearVelocity = MetersPerSecond.zero(),
    tolerance: Pose2d = TOLERANCE,
    poseSupplier: () -> Pose2d = { drive.pose },
    atGoalDebounce: Time = Seconds.of(0.1),
    holonomicController: Pair<TunableHolonomicDriveController, String> =
        Pair(controller, DEFAULT_CONTROLLER_NAME),
): Command =
    runOnce({
            controller.setTolerance(tolerance)
            Logger.recordOutput(
                "$ALIGNMENT_LOGGING_PATH/CurrentRunningController",
                holonomicController.second
            )
        })
        .andThen(
            run({
                    drive.runVelocity(
                        holonomicController.first.calculate(
                            poseSupplier.invoke(),
                            goalPose,
                            linearVelocity.`in`(MetersPerSecond),
                            goalPose.rotation
                        )
                    )
                })
                .until(
                    Trigger { controller.atReference() }
                        .debounce(atGoalDebounce.`in`(Seconds))
                )
        )
        .withName("Drive/AlignToPose")

fun profiledAlignToPose(
    goalPose: Pose2d,
    linearVelocity: LinearVelocity = MetersPerSecond.zero(),
    tolerance: Pose2d = TOLERANCE,
    poseSupplier: () -> Pose2d = { drive.pose },
    atGoalDebounce: Time = 0.1.sec,
): Command =
    runOnce({
        profiledController.poseSupplier = poseSupplier
        Logger.recordOutput(
            "$ALIGNMENT_LOGGING_PATH/CurrentRunningController",
            "ProfiledController"
        )
    }).andThen(
        run({
            drive.runVelocity(
                profiledController.calculate(goalPose)
            )
        }).until(
            profiledController.atGoal.debounce(atGoalDebounce[sec])
        ).withName("Drive/ProfiledAlignToPose")
    )

