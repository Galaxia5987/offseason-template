package org.team5987.annotation

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
import com.squareup.kotlinpoet.*

// Call `LoggedRegistry.registerAll()` on robot init!!!!!!

class LoggedOutputProcessor(
    private val codeGenerator: CodeGenerator,
    private val logger: KSPLogger
) : SymbolProcessor {

    override fun process(resolver: Resolver): List<KSAnnotated> {
        logger.info("LoggedOutputProcessor started processing...")
        val symbols = resolver.getSymbolsWithAnnotation("org.team5987.annotation.LoggedOutput")

        if (symbols.none()) {
            logger.warn("No @LoggedOutput symbols found this round.")
            return emptyList()
        }

        logger.info("Found ${symbols.count()} @LoggedOutput symbols.")

        val fileSpecBuilder = FileSpec.builder("frc.robot.lib.logged_output.generated", "LoggedRegistry")
        val funSpecBuilder = FunSpec.builder("registerAllLoggedOutputs")
            .addStatement("// [LoggedOutputManager] registers all LoggedOutput fields and methods")

        fileSpecBuilder.addImport("frc.robot.lib.logged_output", "LoggedOutputManager")

        for (symbol in symbols) {
            val key = symbol.annotations
                .first { it.shortName.asString() == "LoggedOutput" }
                .arguments.first().value.toString()

            when (symbol) {
                is KSPropertyDeclaration -> { // or is property getter
                    val className = symbol.parentDeclaration?.qualifiedName?.asString() ?: continue
                    val packageName = className.substringBeforeLast(".")
                    val simpleName = className.substringAfterLast(".")
                    val classType = ClassName(packageName, simpleName)
                    val fieldName = symbol.simpleName.asString()

                    logger.info("Registering field: $className.$fieldName with key=$key")

                    // LoggedOutputManager.registerField("key", MyClass::myField)
                    funSpecBuilder.addStatement(
                        "LoggedOutputManager.registerField(%S, %T::%L)",
                        key,
                        classType,
                        fieldName
                    )
                }


                is KSFunctionDeclaration -> {
                    val methodName = symbol.simpleName.asString()
                    val parentDecl = symbol.parentDeclaration

                    if (parentDecl == null) {
                        // TOP-LEVEL FUNCTION
                        val pkg = symbol.containingFile?.packageName?.asString() ?: continue
                        val member = MemberName(pkg, methodName)

                        logger.info("Registering TOP-LEVEL method: $pkg.$methodName with key=$key")

                        // LoggedOutputManager.registerMethod("key", ::testFun)
                        funSpecBuilder.addStatement(
                            "LoggedOutputManager.registerMethod(%S, ::%M)",
                            key,
                            member
                        )
                    } else {
                        // METHOD INSIDE TYPE (object/companion/@JvmStatic works as KFunction0 if no receiver)
                        val classFqName = parentDecl.qualifiedName?.asString() ?: continue
                        val packageName = classFqName.substringBeforeLast(".")
                        val simpleName = classFqName.substringAfterLast(".")
                        val classType = ClassName(packageName, simpleName)

                        logger.info("Registering MEMBER method: $classFqName.$methodName with key=$key")

                        // LoggedOutputManager.registerMethod("key", MyType::methodName)
                        funSpecBuilder.addStatement(
                            "LoggedOutputManager.registerMethod(%S, %T::%L)",
                            key,
                            classType,
                            methodName
                        )
                    }
                }
            }
        }

        fileSpecBuilder.addFunction(funSpecBuilder.build())

        logger.info("Writing generated file: LoggedRegistry.kt")

        val file = codeGenerator.createNewFile(
            Dependencies.ALL_FILES,
            "frc.robot.lib.logged_output.generated",
            "LoggedRegistry"
        )

        file.bufferedWriter().use { writer ->
            fileSpecBuilder.build().writeTo(writer)
        }

        logger.info("LoggedOutputProcessor finished generating LoggedRegistry.kt")

        return emptyList()
    }
}

class LoggedOutputProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return LoggedOutputProcessor(environment.codeGenerator, environment.logger)
    }
}
