package frc.robot.lib.controllers

import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.lib.extensions.*
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class TunableProfiledDriveController(
    private val translationGains: PIDConstants,
    private val translationConstraints: Constraints,
    private val translationTolerance: Distance = 0.1.m,
    private val rotationGains: PIDConstants,
    private val rotationConstraints: Constraints,
    private val rotationTolerance: Angle = 1.deg,
    private val poseSupplier: () -> Pose2d,
    private val speedsSupplier: () -> ChassisSpeeds,
) {
    private val translationKP = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/kP", translationGains.kP)
    private val translationKI = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/kI", translationGains.kI)
    private val translationKD = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/kD", translationGains.kD)
    private val maxTranslationalVelocity = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/maxVelocity", translationConstraints.maxVelocity)
    private val maxTranslationalAcceleration = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/maxAcceleration", translationConstraints.maxAcceleration)
    private val loggedTranslationTolerance = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/tolerance", translationTolerance[m])

    private val rotationKP = LoggedNetworkNumber("$ROTATION_TUNING_PATH/kP", rotationGains.kP)
    private val rotationKI = LoggedNetworkNumber("$ROTATION_TUNING_PATH/kI", rotationGains.kI)
    private val rotationKD = LoggedNetworkNumber("$ROTATION_TUNING_PATH/kD", rotationGains.kD)
    private val maxRotationalVelocity = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/maxVelocity", rotationConstraints.maxVelocity)
    private val maxRotationalAcceleration = LoggedNetworkNumber("$TRANSLATION_TUNING_PATH/maxAcceleration", rotationConstraints.maxAcceleration)
    private val loggedRotationTolerance = LoggedNetworkNumber("$ROTATION_TUNING_PATH/tolerance", rotationTolerance[deg])


    private lateinit var xController : ProfiledPIDController
    private lateinit var yController : ProfiledPIDController
    private lateinit var rotationController : ProfiledPIDController

    init {
        initializeControllers()
    }

    private fun initializeControllers() {
        xController = ProfiledPIDController(
            translationKP.get(),
            translationKI.get(),
            translationKD.get(),
            Constraints(
                maxTranslationalVelocity.get(), maxRotationalVelocity.get()
            )
        )
        yController = ProfiledPIDController(
            translationKP.get(),
            translationKI.get(),
            translationKD.get(),
            Constraints(
                maxTranslationalVelocity.get(), maxTranslationalAcceleration.get()
            )
        )
        rotationController = ProfiledPIDController(
            rotationKP.get(),
            rotationKI.get(),
            rotationKD.get(),
            Constraints(
                maxRotationalVelocity.get(), maxRotationalAcceleration.get()
            )
        )

        xController.setTolerance(loggedTranslationTolerance.get())
        yController.setTolerance(loggedTranslationTolerance.get())
        rotationController.setTolerance(loggedRotationTolerance.get().deg[rad])
    }

    fun reset() {
        val currentPose = poseSupplier.invoke()
        val currentSpeeds = speedsSupplier.invoke()

        initializeControllers()
        xController.reset(currentPose.x, currentSpeeds.vxMetersPerSecond)
        yController.reset(currentPose.y, currentSpeeds.vyMetersPerSecond)
        rotationController.reset(currentPose.rotation.radians, currentSpeeds.omegaRadiansPerSecond)
    }

    private fun log() {
        xController.log("ProfiledXController")
        yController.log("ProfiledYController")
        rotationController.log("ProfiledRotationController")
    }

    fun calculate(
        goalPose: Pose2d,
        endVelocity: LinearVelocity = 0.mps,
    ): ChassisSpeeds {
        val currentPose = poseSupplier.invoke()
        val outputSpeeds = ChassisSpeeds(
            xController.calculate(currentPose.x),
            yController.calculate(currentPose.y),
            rotationController.calculate(currentPose.rotation.radians)
        )

        log()
        Logger.recordOutput("$ALIGNMENT_LOGGING_PATH/Profiled/ControllerOutputSpeeds", outputSpeeds)

        return outputSpeeds
    }
}