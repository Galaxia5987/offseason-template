package frc.robot.lib.controllers

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Vector
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N6
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.*
import frc.robot.LOOP_TIME
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.TunerConstants.*
import org.littletonrobotics.junction.Logger

data class PoseState(
    val pose: Pose2d,
    val speeds: ChassisSpeeds = ChassisSpeeds()
)

private fun Double.toTrapezoidState(speed: Double) =
    TrapezoidProfile.State(this, speed)

private fun PoseState.x() = pose.x.toTrapezoidState(speeds.vxMetersPerSecond)

private fun PoseState.y() = pose.y.toTrapezoidState(speeds.vyMetersPerSecond)

private fun PoseState.theta() =
    pose.rotation.radians.toTrapezoidState(speeds.omegaRadiansPerSecond)

// Physical properties of the robot
private const val m = 50.0 // Robot mass in kg
private const val Izz =
    2.5 // Moment of inertia, tuned based on turn performance


private const val MAT_B_LINEAR_SCALING = 2000
private const val MAT_B_ANGULAR_SCALING = 500

// Default traction values
private const val tractionFactor = 0.9 // Less than 1.0 accounts for wheel slip
private const val rotationTractionFactor = 0.85 // Rotational slip factor

// TODO: Add feedforward
class ModelPredictiveController(private val drive: Drive) {

    // Define A, B, C, D matrices for the swerve drivetrain system
    private val matA: Matrix<N6, N6> =
        Matrix(N6.instance, N6.instance).apply {
            // Positions depend on previous positions
            this[0, 0] = 1.0
            this[1, 1] = 1.0
            this[2, 2] = 1.0

            // Positions are updated by velocities over time
            this[0, 3] = LOOP_TIME
            this[1, 4] = LOOP_TIME
            this[2, 5] = LOOP_TIME

            // Velocity damping due to friction
            this[3, 3] = 0.8 // X velocity damping
            this[4, 4] = 0.8 // Y velocity damping
            this[5, 5] = 0.8 // Theta velocity damping
        }

    // Adjusted B Matrix
    private val matB: Matrix<N6, N3> =
        Matrix(N6.instance, N3.instance).apply {
            // X acceleration input (affected by mass and traction)
            this[3, 0] = LOOP_TIME / (m * tractionFactor) * MAT_B_LINEAR_SCALING

            // Y acceleration input (affected by mass and traction)
            this[4, 1] = LOOP_TIME / (m * tractionFactor) * MAT_B_LINEAR_SCALING

            // Theta acceleration input (affected by moment of inertia)
            this[5, 2] = LOOP_TIME / (Izz * rotationTractionFactor) * MAT_B_ANGULAR_SCALING
        }

    private val matC: Matrix<N3, N6> =
        Matrix(N3.instance, N6.instance).apply {
            this[0, 0] = 1.0 // Measurement of x position
            this[1, 1] = 1.0 // Measurement of y position
            this[2, 2] = 1.0 // Measurement of rotation (theta)
        }

    private val matD: Matrix<N3, N3> =
        Matrix(N3.instance, N3.instance).apply {
            // Since this is a swerve drivetrain the output doesn't have an immediate effect on
            // the input, hence this matrix is empty.
        }

    private val linearSystem: LinearSystem<N6, N3, N3> =
        LinearSystem(matA, matB, matC, matD)

    // Create the state cost matrix for N6 dimensions
    private val stateCostMatrix: Matrix<N6, N1> = // Q Matrix
        Matrix(N6.instance, N1.instance).apply {
            this[0, 0] = 1.0 // x position cost
            this[1, 0] = 1.0 // y position cost
            this[2, 0] = 1.0 // theta position cost
            this[3, 0] = 1.0 // x velocity cost
            this[4, 0] = 1.0 // y velocity cost
            this[5, 0] = 1.0 // theta velocity cost
        }

    // Create the control cost matrix for N3 dimensions
    private val controlCostMatrix: Matrix<N3, N1> = // R Matrix
        Matrix(N3.instance, N1.instance).apply {
            this[0, 0] = 0.1 // x velocity control effort
            this[1, 0] = 0.1 // y velocity control effort
            this[2, 0] = 0.1 // theta velocity control effort
        }

    // Linear Quadratic Regulator (LQR) for state control
    private val lqr: LinearQuadraticRegulator<N6, N3, N3> =
        LinearQuadraticRegulator(
            linearSystem,
            Vector(stateCostMatrix), // State cost
            Vector(controlCostMatrix), // Control effort cost
            LOOP_TIME
        )

    // Motion profile to ensure smooth trajectory following
    private val xProfile =
        TrapezoidProfile(
            TrapezoidProfile.Constraints(
                kSpeedAt12Volts.`in`(MetersPerSecond),
                kMaxAcceleration.`in`(MetersPerSecondPerSecond)
            )
        )
    private val yProfile =
        TrapezoidProfile(
            TrapezoidProfile.Constraints(
                kSpeedAt12Volts.`in`(MetersPerSecond),
                kMaxAcceleration.`in`(MetersPerSecondPerSecond)
            )
        )
    private val thetaProfile =
        TrapezoidProfile(
            TrapezoidProfile.Constraints(
                kOmegaSpeedAt12Volts.`in`(RadiansPerSecond),
                kOmegaMaxAcceleration.`in`(RadiansPerSecondPerSecond)
            )
        )

    fun update(goalState: PoseState): ChassisSpeeds {
        // Get current robot pose and speed
        val currentPose = drive.pose // Get the robot’s current position
        val currentSpeeds = drive.chassisSpeeds // Get current velocity

        val currentXState: TrapezoidProfile.State =
            currentPose.x.toTrapezoidState(currentSpeeds.vxMetersPerSecond)
        val currentYState: TrapezoidProfile.State =
            currentPose.y.toTrapezoidState(currentSpeeds.vyMetersPerSecond)
        val currentThetaState: TrapezoidProfile.State =
            currentPose.rotation.radians.toTrapezoidState(
                currentSpeeds.omegaRadiansPerSecond
            )

        // Generate motion profiles for X, Y, and Rotation (theta)
        val xSetpoint: TrapezoidProfile.State =
            xProfile.calculate(LOOP_TIME, currentXState, goalState.x())
        val ySetpoint: TrapezoidProfile.State =
            yProfile.calculate(LOOP_TIME, currentYState, goalState.y())
        val thetaSetpoint: TrapezoidProfile.State =
            thetaProfile.calculate(
                LOOP_TIME,
                currentThetaState,
                goalState.theta()
            )

        println("matB values: ${matB.storage}")

        // Compute error states
        val errorState: Matrix<N6, N1> =
            Matrix(N6.instance, N1.instance).apply {
                this[0, 0] = xSetpoint.position - currentPose.x // X Error
                this[1, 0] = ySetpoint.position - currentPose.y // Y Error
                this[2, 0] =
                    thetaSetpoint.position -
                        currentPose.rotation.radians // Theta Error
                this[3, 0] =
                    xSetpoint.velocity -
                        currentSpeeds.vxMetersPerSecond // X Velocity Error
                this[4, 0] =
                    ySetpoint.velocity -
                        currentSpeeds.vyMetersPerSecond // Y Velocity Error
                this[5, 0] =
                    thetaSetpoint.velocity -
                        currentSpeeds
                            .omegaRadiansPerSecond // Theta Velocity Error
            }

        // Compute optimal control input using the LQR
        val controlInput: Matrix<N3, N1> = lqr.calculate(errorState)
        println("LQR Gain Matrix K: ${lqr.k}")

        // Return the calculated output
        val chassisSpeeds =
            ChassisSpeeds(
                controlInput[0, 0],
                controlInput[1, 0],
                controlInput[2, 0]
            )
        Logger.recordOutput("StateSpace/DesiredSpeeds", chassisSpeeds)
        return chassisSpeeds
    }
}
