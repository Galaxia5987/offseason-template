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
import frc.robot.lib.extensions.div
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

private val TIME_BETWEEN_ROUTINES = 1.sec

/**
 * Extension function that creates a [SysIdCommand] for any subsystem that
 * implements [SysIdable] and [SubsystemBase].
 *
 * @param forwardRoutineConfig configuration for the forward routine.
 * @param backwardRoutineConfig configuration for the backward routine.
 *
 * @return A [SysIdCommand] instance.
 */
fun <T> T.createSysIdCommand(
    forwardRoutineConfig: SysIdCommand<T>.LoggedSysIdRoutineConfig,
    backwardRoutineConfig: SysIdCommand<T>.LoggedSysIdRoutineConfig
): SysIdCommand<T> where T : SysIdable, T : SubsystemBase {
    return SysIdCommand(this, forwardRoutineConfig, backwardRoutineConfig)
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
        get() = { setVoltage(it) }

    /** Applies the specified [voltage] to the subsystem. */
    fun setVoltage(voltage: Voltage)
}

/**
 * Helper that generates a WPILib [Command] to run SysId routines
 * (forward/backward/quasistatic).
 *
 * @param T The subsystem type, which must implement [SysIdable] and extend
 * [SubsystemBase].
 * @property subsystem The target subsystem being characterized.
 * @property forwardRoutineConfig configuration for the forward routine.
 * @property backwardRoutineConfig configuration for the backward routine.
 */
class SysIdCommand<T>
internal constructor(
    private val subsystem: T,
    private var forwardRoutineConfig: LoggedSysIdRoutineConfig,
    private var backwardRoutineConfig: LoggedSysIdRoutineConfig
) where T : SysIdable, T : SubsystemBase {

    private val name = subsystem.name

    private lateinit var forwardRoutine: SysIdRoutine
    private lateinit var backwardRoutine: SysIdRoutine

    /**
     * Sets the config to be used for the **forward** SysId routine. The config
     * includes rampRate, stepVoltage, and timeout.
     *
     * @param forwardRoutineConfig Configuration for the forward routine.
     * @param backwardRoutineConfig Configuration fo the backward routine.
     * @return This [SysIdCommand] instance for chaining.
     */
    fun withRoutineConfig(
        forwardRoutineConfig: LoggedSysIdRoutineConfig,
        backwardRoutineConfig: LoggedSysIdRoutineConfig
    ): SysIdCommand<T> {
        this.forwardRoutineConfig = forwardRoutineConfig
        this.backwardRoutineConfig = backwardRoutineConfig
        return this
    }

    /**
     * Creates the [SysIdRoutine] object with the provided configuration.
     *
     * @param routineConfig A configuration for the routine.
     */
    private fun createRoutine(routineConfig: LoggedSysIdRoutineConfig) =
        SysIdRoutine(
            SysIdRoutine.Config(
                routineConfig.loggedRampRate.get().volts / sec,
                routineConfig.loggedStepVoltage.get().volts,
                routineConfig.loggedTimeout.get().sec,
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
    private fun createRoutineCommands(): Command =
        subsystem.runOnce {
            forwardRoutine = createRoutine(forwardRoutineConfig)
            backwardRoutine = createRoutine(backwardRoutineConfig)
        }

    /**
     * Builds the full characterization command sequence:
     * 1. Initializes routines
     * 2. Runs forward dynamic
     * 3. Runs backward dynamic
     * 4. Runs forward quasistatic
     * 5. Runs backward quasistatic
     *
     * Waits `TIME_BETWEEN_ROUTINES` between each step.
     *
     * @return The full [Command] sequence.
     */
    fun createSysIdCommand(): Command {
        val wait = Commands.waitTime(TIME_BETWEEN_ROUTINES)
        return subsystem
            .defer {
                Commands.sequence(
                    createRoutineCommands(),
                    forwardRoutine.dynamic(SysIdRoutine.Direction.kForward),
                    wait,
                    backwardRoutine.dynamic(SysIdRoutine.Direction.kReverse),
                    wait,
                    forwardRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                    wait,
                    backwardRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
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
    inner class LoggedSysIdRoutineConfig(
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
