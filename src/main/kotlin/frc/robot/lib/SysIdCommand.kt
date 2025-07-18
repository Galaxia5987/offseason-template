package frc.robot.lib

import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

private val TIME_BETWEEN_ROUTINES = 1.sec

/**
 * Extension function that creates a [SysIdCommand] for any subsystem that
 * implements [SysIdable] and [SubsystemBase].
 */
fun <T> T.createSysIDCommand(): SysIdCommand<T> where
T : SysIdable,
T : SubsystemBase {
    return SysIdCommand(this)
}

/**
 * Interface that allows a subsystem to be characterized via SysId. Must provide
 * a method to set voltage on the subsystem.
 */
interface SysIdable {
    /**
     * Function that consumes a voltage and applies it to the subsystem.
     * Defaults to using [setVoltage].
     */
    val setVoltageConsumer: (Voltage) -> Unit
        get() = { voltage: Voltage -> setVoltage(voltage) }

    /** Applies the specified [voltage] to the subsystem. */
    fun setVoltage(voltage: Voltage)
}

/**
 * A builder-style helper that generates a WPILib [Command] to run SysId
 * routines (forward/backward/quasistatic).
 *
 * @param T The subsystem type, which must implement both [SysIdable] and
 * [SubsystemBase].
 * @property subsystem The target subsystem being characterized.
 */
class SysIdCommand<T> internal constructor(private val subsystem: T) where
T : SysIdable,
T : SubsystemBase {
    private val name = subsystem::class.simpleName
    private var forwardRoutineConstants: LoggedSysIdRoutineConstants? = null
    private var backwardRoutineConstants: LoggedSysIdRoutineConstants? = null

    private var routineForwards: SysIdRoutine? = null
    private var routineBackwards: SysIdRoutine? = null

    /**
     * Sets the constants to be used for the **forward** SysId routine.
     *
     * @param forwardRoutineConstants Constants such as ramp rate, step voltage,
     * and timeout.
     * @return This [SysIdCommand] instance for chaining.
     */
    fun withForwardRoutine(
        forwardRoutineConstants: LoggedSysIdRoutineConstants
    ): SysIdCommand<T> {
        this.forwardRoutineConstants = forwardRoutineConstants
        return this
    }

    /**
     * Sets the constants to be used for the **backward** SysId routine.
     *
     * @param backwardRoutineConstants Constants such as ramp rate, step
     * voltage, and timeout.
     * @return This [SysIdCommand] instance for chaining.
     */
    fun withBackwardRoutine(
        backwardRoutineConstants: LoggedSysIdRoutineConstants
    ): SysIdCommand<T> {
        this.backwardRoutineConstants = backwardRoutineConstants
        return this
    }

    /**
     * Sets the constants to be used for the **backward** SysId routine.
     *
     * @param backwardRoutineConstants Constants such as ramp rate, step
     * voltage, and timeout.
     * @return This [SysIdCommand] instance for chaining.
     */
    private fun createRoutine(routineConstants: LoggedSysIdRoutineConstants) =
        SysIdRoutine(
            SysIdRoutine.Config(
                routineConstants.loggedRampRate.get().volts.per(sec),
                routineConstants.loggedStepVoltage.get().volts,
                routineConstants.loggedTimeout.get().sec,
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput("$name/state", state.toString())
            },
            SysIdRoutine.Mechanism(
                subsystem.setVoltageConsumer,
                null,
                subsystem
            )
        )

    /** Initializes the internal SysId routines from the stored constants. */
    private fun createRoutinesCommand(): Command =
        subsystem.runOnce {
            requireNotNull(forwardRoutineConstants) {
                "Forward routine constants not initialized!"
            }
            requireNotNull(backwardRoutineConstants) {
                "Backward routine constants not initialized!"
            }

            routineForwards = createRoutine(forwardRoutineConstants!!)
            routineBackwards = createRoutine(backwardRoutineConstants!!)
        }

    /**
     * Builds the full characterization command sequence:
     * 1. Initializes routines
     * 2. Runs forward dynamic
     * 3. Runs backward dynamic
     * 4. Runs forward quasistatic
     * 5. Runs backward quasistatic
     *
     * Waits 1 second between each step.
     *
     * @return The full [Command] sequence.
     */
    fun createSysIDCommand(): Command {
        return subsystem
            .defer {
                Commands.sequence(
                    createRoutinesCommand(),
                    routineForwards!!.dynamic(SysIdRoutine.Direction.kForward),
                    Commands.waitSeconds(TIME_BETWEEN_ROUTINES[sec]),
                    routineBackwards!!.dynamic(SysIdRoutine.Direction.kReverse),
                    Commands.waitSeconds(TIME_BETWEEN_ROUTINES[sec]),
                    routineForwards!!.quasistatic(
                        SysIdRoutine.Direction.kForward
                    ),
                    Commands.waitSeconds(TIME_BETWEEN_ROUTINES[sec]),
                    routineBackwards!!.quasistatic(
                        SysIdRoutine.Direction.kReverse
                    )
                )
            }
            .withName("$name/characterize")
    }

    /**
     * Holds the constants used for configuring a [SysIdRoutine], with tunable
     * logging support.
     *
     * @param rampRate The ramp rate for quasistatic tests.
     * @param stepVoltage The voltage step size for dynamic tests.
     * @param timeout The timeout after which the routine will stop.
     */
    inner class LoggedSysIdRoutineConstants(
        private val rampRate: Velocity<VoltageUnit>,
        private val stepVoltage: Voltage,
        private val timeout: Time
    ) {
        /** Logged ramp rate value in volts/sec, tunable via NetworkTables. */
        val loggedRampRate =
            LoggedNetworkNumber(
                "/Tuning/$name/rampRate",
                rampRate.`in`(volts.per(sec))
            )

        /** Logged step voltage in volts, tunable via NetworkTables. */
        val loggedStepVoltage =
            LoggedNetworkNumber("/Tuning/$name/stepVoltage", stepVoltage[volts])

        /** Logged timeout duration in seconds, tunable via NetworkTables. */
        val loggedTimeout =
            LoggedNetworkNumber("/Tuning/$name/timeout", timeout[sec])
    }
}
