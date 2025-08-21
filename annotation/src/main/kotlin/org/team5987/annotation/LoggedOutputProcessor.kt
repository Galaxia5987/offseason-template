package org.team5987.annotation

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
import com.squareup.kotlinpoet.*
import java.util.function.Supplier

// Call to `LoggedRegistry.registerAll` on robot init!!!!!!

class LoggedOutputProcessor(
    private val codeGenerator: CodeGenerator,
    private val logger: KSPLogger
) : SymbolProcessor {

    override fun process(resolver: Resolver): List<KSAnnotated> {
        logger.warn("LoggedOutputProcessor started processing...")
        val symbols = resolver.getSymbolsWithAnnotation("org.team5987.annotation.LoggedOutput")

        if (symbols.none()) {
            logger.warn("No @LoggedOutput symbols found this round.")
            return emptyList()
        }

        logger.warn("Found ${symbols.count()} @LoggedOutput symbols.")

        val fileSpecBuilder = FileSpec.builder("frc.robot.lib.logged_output.generated", "LoggedRegistry")
        val funSpecBuilder = FunSpec.builder("registerAll")
            .addStatement("// Auto-generated: registers all LoggedOutput fields and methods")

        fileSpecBuilder.addImport("frc.robot.lib.logged_output", "LoggedOutputManager")

        for (symbol in symbols) {
            val key = symbol.annotations
                .first { it.shortName.asString() == "LoggedOutput" }
                .arguments.first().value.toString()


            when (symbol) {
                is KSPropertyDeclaration -> {
                    val className = symbol.parentDeclaration?.qualifiedName?.asString() ?: continue
                    val fieldName = symbol.simpleName.asString()

                    val packageName = className.substringBeforeLast(".")
                    val simpleName = className.substringAfterLast(".")
                    val classType = ClassName(packageName, simpleName)

                    logger.warn("Registering field: $className.$fieldName with key=$key")

                    funSpecBuilder.addStatement(
                        "LoggedOutputManager.registerField(%S, %T::class.java, %T { %L.%L })",
                        key,
                        classType,
                        Supplier::class,
                        classType,
                        fieldName
                    )
                }
                is KSFunctionDeclaration -> {
                    val className = symbol.parentDeclaration?.qualifiedName?.asString()
                        ?: symbol.containingFile?.packageName?.asString() ?: continue
                    val methodName = symbol.simpleName.asString()

                    val packageName = className.substringBeforeLast(".")
                    val simpleName = className.substringAfterLast(".")
                    val classType = ClassName(packageName, simpleName)

                    logger.warn("Registering method: $className.$methodName() with key=$key")

                    funSpecBuilder.addStatement(
                        "LoggedOutputManager.registerMethod(%S, %T::class.java, %T { %L.%L() })",
                        key,
                        classType,
                        Supplier::class,
                        classType,
                        methodName
                    )
                }
            }
        }

        fileSpecBuilder.addFunction(funSpecBuilder.build())

        logger.warn("Writing generated file: LoggedRegistry.kt")

        // Write file
        val file = codeGenerator.createNewFile(
            Dependencies.ALL_FILES,
            "frc.robot.lib.logged_output.generated",
            "LoggedRegistry"
        )

        file.bufferedWriter().use { writer ->
            fileSpecBuilder.build().writeTo(writer)
        }

        logger.warn("LoggedOutputProcessor finished generating LoggedRegistry.kt")

        return emptyList()
    }
}

class LoggedOutputProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return LoggedOutputProcessor(environment.codeGenerator, environment.logger)
    }
}