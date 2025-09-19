package frc.robot.lib.logged_output

import edu.wpi.first.units.Measure
import edu.wpi.first.util.WPISerializable
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.toPrimitiveTypeJava
import java.util.function.*
import kotlin.reflect.KFunction
import kotlin.reflect.KProperty0
import kotlin.reflect.jvm.javaGetter
import kotlin.reflect.jvm.javaMethod
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d

object LoggedOutputManager : SubsystemBase() {
    private val callbacks = mutableListOf<Runnable>()

    override fun periodic() {
        callbacks.forEach { it.run() }
    }

    private fun makeKey(
        key: String,
        name: String,
        declaringClass: String?
    ): String {
        return key.ifBlank { "${declaringClass ?: "<unknown>"}//$name" }
    }

    fun <T> registerField(key: String, property: KProperty0<T>) {
        val declaringClass = property.javaGetter?.declaringClass?.simpleName
        val actualKey = makeKey(key, property.name, declaringClass)
        register(actualKey, property::get)
    }

    fun <T> registerMethod(key: String, function: KFunction<T>) {
        if (function.parameters.isNotEmpty()) {
            throw IllegalArgumentException(
                "Only zero-arg functions are supported: $key"
            )
        }

        val declaringClass =
            function.javaMethod?.declaringClass?.simpleName ?: "<top-level>"
        val actualKey = makeKey(key, function.name, declaringClass)
        register(actualKey, function::call)
    }

    // Taken from advantageKit's `AutoLogOutputManager`,
    // https://github.com/rakrakon/AdvantageKit/blob/main/akit/src/main/java/org/littletonrobotics/junction/AutoLogOutputManager.java
    @Suppress("UNCHECKED_CAST")
    private fun register(key: String, supplier: Supplier<*>) {
        val type = supplier.get()::class.java.toPrimitiveTypeJava()!!

        if (!type.isArray) {
            // Single types
            if (type == Boolean::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Boolean)
                    }
                )
            } else if (type == Int::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Int)
                    }
                )
            } else if (type == Long::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Long)
                    }
                )
            } else if (type == Float::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Float)
                    }
                )
            } else if (type == Double::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Double)
                    }
                )
            } else if (type == String::class.java) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as String?)
                    }
                )
            } else if (type.isEnum) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (
                            value != null
                        ) // Cannot cast to enum subclass, log the name directly
                         Logger.recordOutput(key, (value as Enum<*>).name)
                    }
                )
            } else if (BooleanSupplier::class.java.isAssignableFrom(type)) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as BooleanSupplier?)
                    }
                )
            } else if (IntSupplier::class.java.isAssignableFrom(type)) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as IntSupplier?)
                    }
                )
            } else if (LongSupplier::class.java.isAssignableFrom(type)) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as LongSupplier?)
                    }
                )
            } else if (DoubleSupplier::class.java.isAssignableFrom(type)) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as DoubleSupplier?)
                    }
                )
            } else if (Measure::class.java.isAssignableFrom(type)) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Measure<*>?)
                    }
                )
            } else if (type == LoggedMechanism2d::class.java) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as LoggedMechanism2d?
                            )
                    }
                )
            } else if (type.isRecord) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Record)
                    }
                )
            } else {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            try {
                                Logger.recordOutput(
                                    key,
                                    value as WPISerializable
                                )
                            } catch (e: ClassCastException) {
                                DriverStation.reportError(
                                    "[LoggedOutputManager] Auto serialization is not supported for type " +
                                        type.getSimpleName(),
                                    false
                                )
                            }
                    }
                )
            }
        } else if (!type.componentType.isArray) {
            // Array types
            val componentType = type.componentType
            if (componentType == Byte::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as ByteArray?)
                    }
                )
            } else if (componentType == Boolean::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as BooleanArray?)
                    }
                )
            } else if (componentType == Int::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as IntArray?)
                    }
                )
            } else if (componentType == Long::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as LongArray?)
                    }
                )
            } else if (componentType == Float::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as FloatArray?)
                    }
                )
            } else if (componentType == Double::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as DoubleArray?)
                    }
                )
            } else if (componentType == String::class.java) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Array<String?>?)
                    }
                )
            } else if (componentType.isEnum) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null) {
                            // Cannot cast to enum subclass, log the names directly
                            val enumValue = value as Array<Enum<*>>
                            val names = arrayOfNulls<String>(enumValue.size)
                            for (i in enumValue.indices) {
                                names[i] = enumValue[i].name
                            }
                            Logger.recordOutput(key, names)
                        }
                    }
                )
            } else if (componentType.isRecord) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, *value as Array<Record?>)
                    }
                )
            } else {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null) {
                            try {
                                Logger.recordOutput(
                                    key,
                                    *value as Array<StructSerializable?>
                                )
                            } catch (e: ClassCastException) {
                                DriverStation.reportError(
                                    "[LoggedOutputManager] Auto serialization is not supported for array type " +
                                        componentType.getSimpleName(),
                                    false
                                )
                            }
                        }
                    }
                )
            }
        } else {
            // 2D array types
            val componentType = type.componentType.componentType
            if (componentType == Byte::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<ByteArray?>?
                            )
                    }
                )
            } else if (componentType == Boolean::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<BooleanArray?>?
                            )
                    }
                )
            } else if (componentType == Int::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(key, value as Array<IntArray?>?)
                    }
                )
            } else if (componentType == Long::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<LongArray?>?
                            )
                    }
                )
            } else if (componentType == Float::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<FloatArray?>?
                            )
                    }
                )
            } else if (componentType == Double::class.javaPrimitiveType) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<DoubleArray?>?
                            )
                    }
                )
            } else if (componentType == String::class.java) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<Array<String?>?>?
                            )
                    }
                )
            } else if (componentType.isEnum) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null) {
                            // Cannot cast to enum subclass, log the names directly
                            val enumValue = value as Array<Array<Enum<*>>>
                            val names: Array<Array<String?>?> =
                                arrayOfNulls(enumValue.size)
                            for (row in enumValue.indices) {
                                val rowValue = enumValue[row]
                                names[row] = arrayOfNulls(rowValue.size)
                                for (column in rowValue.indices) {
                                    names[row]?.set(
                                        column,
                                        rowValue[column].name
                                    )
                                }
                            }
                            Logger.recordOutput(key, names)
                        }
                    }
                )
            } else if (componentType.isRecord) {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null)
                            Logger.recordOutput(
                                key,
                                value as Array<Array<Record>?>?
                            )
                    }
                )
            } else {
                callbacks.add(
                    Runnable {
                        val value = supplier.get()
                        if (value != null) {
                            try {
                                Logger.recordOutput(
                                    key,
                                    value as Array<Array<StructSerializable>?>?
                                )
                            } catch (e: ClassCastException) {
                                DriverStation.reportError(
                                    ("[LoggedOutputManager] Auto serialization is not supported for 2D array type " +
                                        componentType.getSimpleName()),
                                    false
                                )
                            }
                        }
                    }
                )
            }
        }
    }
}
