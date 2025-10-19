package org.team5987.annotation

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.processing.CodeGenerator
import com.google.devtools.ksp.symbol.*
import com.squareup.kotlinpoet.*
import com.squareup.kotlinpoet.ksp.writeTo

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
            generateEmptyFile(codeGenerator, resolver)
            return emptyList()
        }

        logger.info("Found ${symbols.count()} @LoggedOutput symbols.")

        val fileSpecBuilder = FileSpec.builder("frc.robot.lib.logged_output.generated", "LoggedRegistry")
        val funSpecBuilder = FunSpec.builder("registerAllLoggedOutputs")
            .addStatement("// [LoggedOutputManager] registers all LoggedOutput fields and methods")

        fileSpecBuilder.addImport("frc.robot.lib.logged_output", "LoggedOutputManager")

        for (symbol in symbols) {
            val baseVal = symbol.annotations
                .first { it.shortName.asString() == "LoggedOutput" }
                .arguments
            val key by lazy {
                if (baseVal.first().name?.asString() == "key")
                    baseVal.first().value.toString()
                else baseVal[1].value.toString()
            }

            var path =
                if (baseVal.first().name?.asString() == "path")
                    baseVal.first().value.toString()
                else baseVal[1].value.toString()

            when (symbol) {
                is KSPropertyDeclaration -> { // or is property getter
                    val className = symbol.parentDeclaration?.qualifiedName?.asString()
                    if (className != null) {
                        val packageName = className.substringBeforeLast(".")
                        val simpleName = className.substringAfterLast(".")
                        val classType = ClassName(packageName, simpleName)
                        val fieldName = symbol.simpleName.asString()
                        val owner = symbol.parentDeclaration as? KSClassDeclaration
                        owner?.superTypes?.forEach { classes ->
                            val resolvedType = classes.resolve()
                            val decl = resolvedType.declaration as? KSClassDeclaration
                            if (decl?.qualifiedName?.asString()?.substringAfterLast(".") == "SubsystemBase") {
                                path = path.ifEmpty { "Subsystems/$simpleName" }
                            }
                        }

                        logger.info("Registering field: $className.$fieldName with key=$key")

                        // LoggedOutputManager.registerField("key", MyClass::myField)
                        funSpecBuilder.addStatement(
                            "LoggedOutputManager.registerField(%S, %T::%L, %S)",
                            key,
                            classType,
                            fieldName,
                            path
                        )
                    } else {
                        val methodName = symbol.simpleName.asString()
                        // TOP-LEVEL FUNCTION
                        val pkg = symbol.containingFile?.packageName?.asString() ?: continue
                        val member = MemberName(pkg, methodName)

                        funSpecBuilder.addStatement(
                            "LoggedOutputManager.registerField(%S, ::%M,%S)",
                            key,
                            member,
                            path
                        )
                        logger.info("Registering field: $pkg.$methodName with key=$key")
                    }
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
                            "LoggedOutputManager.registerMethod(%S, ::%M, %S)",
                            key,
                            member,
                            path
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
                            "LoggedOutputManager.registerMethod(%S, %T::%L, %S)",
                            key,
                            classType,
                            methodName,
                            path
                        )
                    }
                }

                is KSClassDeclaration -> {

                    val pkg = symbol.containingFile?.packageName?.asString() ?: continue

                    val className = symbol.simpleName.asString()
                    val classType = ClassName(pkg, className)
                    symbol.getAllProperties().forEach {
                        val methodName = it.simpleName.asString()
                        val pkg = it.containingFile?.packageName?.asString()
                        if (pkg != null) {
                            funSpecBuilder.addStatement(
                                "LoggedOutputManager.registerField(%S, %T::%L, %S)",
                                key,
                                classType,
                                methodName,
                            )
                        }
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

private fun generateEmptyFile(codeGenerator: CodeGenerator, resolver: Resolver) {
    try {
        val pkg = "frc.robot.lib.logged_output.generated"
        val name = "LoggedRegistry"
        val file = FileSpec.builder(pkg, name)
            .addImport("frc.robot.lib.logged_output", "LoggedOutputManager")
            .addFunction(
                FunSpec.builder("registerAllLoggedOutputs")
                    .addKdoc("Auto-generated stub. Safe to call even if no @LoggedOutput symbols were found.\n")
                    .addStatement("// [LoggedOutputManager] registers all LoggedOutput fields and methods")
                    .build()
            )
            .build()
        val deps = Dependencies(
            aggregating = true,
            *resolver.getAllFiles().toList().toTypedArray()
        )
        file.writeTo(codeGenerator, deps)
    } catch (e: FileAlreadyExistsException) {
    }
}

class LoggedOutputProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return LoggedOutputProcessor(environment.codeGenerator, environment.logger)
    }
}
