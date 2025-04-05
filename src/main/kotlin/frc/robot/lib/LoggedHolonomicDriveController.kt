package frc.robot.lib

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import frc.robot.subsystems.drive.controller
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

const val LOGGING_LOCATION = "/Tuning/Alignment"
const val TRANSLATION_LOGGING_LOCATION = "$LOGGING_LOCATION/Translation"
const val ROTATION_LOGGING_LOCATION = "$LOGGING_LOCATION/Rotation"


class LoggedHolonomicDriveController(
    translationController: PIDController,
    thetaController: ProfiledPIDController,
) : HolonomicDriveController(translationController, translationController, thetaController) {
    private val translationKP = LoggedNetworkNumber("$TRANSLATION_LOGGING_LOCATION/kP", xController.p)
    private val translationKI = LoggedNetworkNumber("$TRANSLATION_LOGGING_LOCATION/kI", xController.i)
    private val translationKD = LoggedNetworkNumber("$TRANSLATION_LOGGING_LOCATION/kD", xController.d)
    
    private val rotationKP = LoggedNetworkNumber("$ROTATION_LOGGING_LOCATION/kP", thetaController.p)
    private val rotationKI = LoggedNetworkNumber("$ROTATION_LOGGING_LOCATION/kI", thetaController.i)
    private val rotationKD = LoggedNetworkNumber("$ROTATION_LOGGING_LOCATION/kD", thetaController.d)

    override fun calculate(
        currentPose: Pose2d,
        trajectoryPose: Pose2d,
        desiredLinearVelocityMetersPerSecond: Double,
        desiredHeading: Rotation2d,
    ): ChassisSpeeds {
        updatePIDCoefficients()
        controller.log()

        return super.calculate(currentPose, trajectoryPose, desiredLinearVelocityMetersPerSecond, desiredHeading)
    }

    override fun calculate(
        currentPose: Pose2d, desiredState: Trajectory.State, desiredHeading: Rotation2d,
    ): ChassisSpeeds {
        return calculate(
            currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading
        )
    }

    private fun updatePIDCoefficients() {
        xController.setPID(translationKP.get(), translationKI.get(), translationKD.get())
        yController.setPID(translationKP.get(), translationKI.get(), translationKD.get())
        thetaController.setPID(rotationKP.get(), rotationKI.get(), rotationKD.get())
    }
}