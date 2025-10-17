package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretSpin
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.subsystems.Intake.PARAMS.P_Intake


@Config
class Turret(hardwareMap: HardwareMap) : StateLoggable {

    companion object PARAMS {
        @JvmField // Spin Target
        var spinTarget: Double = 0.0

        @JvmField
        var tiltTarget: Double = 0.0

        // REV HD Hex Motor = 28 CPR, 20:1 gear ratio
        private const val encoderCPR = 28.0
        private const val gearRatio = 20.0
        private const val ticksPerRev = encoderCPR * gearRatio
        @JvmField
        var ticksPerDegree: Double = ticksPerRev / 360.0  // ≈ 1.56 ticks/degree

        // Mechanical limits (adjust to your turret’s safe travel)
        @JvmField
        val SPIN_MIN = -(28.85 * ticksPerDegree).toInt()   // ≈ -45     // we want it to be - 90
        @JvmField
        val SPIN_MID = (0.0 * ticksPerDegree).toInt()   // ≈ 0      // we want it to be 0
        @JvmField
        val SPIN_MAX = (28.85 * ticksPerDegree).toInt()   // ≈ +45      // we want it to be +380

        // Tilt limits (servo normalized range)
//        const val TILT_MIN = 0.0
//        const val TILT_MAX = 1.0
//
//        @JvmField var kP_Tilt = 0.17
//        @JvmField var kI_Tilt = 0.0
//        @JvmField var kD_Tilt = 0.1
    }


//    val spinMotor = hardwareMap.get(DcMotorEx::class.java, "spin")
//    val tiltMotor = hardwareMap.get(DcMotorEx::class.java, "tilt")
//    val spinServo = hardwareMap.get(Servo::class.java, "spin") // servo
//    val spinOutput = hardwareMap.get(AnalogInput::class.java, "spinOutput")
//    val tiltServo = hardwareMap.get(Servo::class.java, "tilt") // servo
//    val tiltOutput = hardwareMap.get(AnalogInput::class.java, "tiltOutput")

    val spinMotor = TurretSpin(
        hardwareMap,
        "spin",
        DcMotorSimple.Direction.FORWARD,
        P_Intake,
        ticksPerDegree
    )

    val spinCurrentPosition get() = spinMotor.currentPosition
    val spinTargetPos get() = spinMotor.targetDistance
//    val tiltCurrentPosition get() = tiltTarget // servo doesn't have encoder


//    private var spinTarget = 0.5  // normalized target
//    private var spinIntegral = 0.0
//    private var tiltTarget = 0.5  // normalized target
//    private var tiltIntegral = 0.0
//    private var lastError = 0.0
//    private val timer = ElapsedTime()


//    private fun initMotor() {
//        spinMotor.targetPosition = 0
//        spinMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        spinMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//    }


    init {
//        initMotor()
//        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        spinMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        // tiltTarget should already be in 0.0..1.0 range
////        tiltServo.position = tiltTarget.coerceIn(TILT_MIN, TILT_MAX)
    }

    private val spinActionWriter = DownsampledWriter("SPIN_ACTION", 50_000_000)
    private val spinPoseWriter = DownsampledWriter("SPIN_POSITION", 50_000_000)
    private val tiltActionWriter = DownsampledWriter("TILT_ACTION", 50_000_000)
    private val tiltPoseWriter = DownsampledWriter("TILT_POSITION", 50_000_000)


    fun resetSpin(): LoggableAction {
        return Loggable("CANCEL_INTAKE", ParallelAction(
            spinMotor.gotoDistance(0.0, 0.1)
        ))
    }


//    @JvmOverloads
//    fun track(positionProvider: TurretSpin.DoubleProvider? = null): LoggableAction {
//        if (positionProvider?.run() == null) return LoggingSequential(
//            "CAPTURE_CLOSE_SAMPLE",
//            Loggable(
//                "SEARCH_AND_FLIP", ParallelAction(
//                    spinMotor.gotoDistance(0.0, 0.1)
//                )
//            ),
//        )
//
//        val holdPositionAction = spinMotor.holdVariablePosition(positionProvider)
//
//        return LoggingSequential(
//            "CAPTURE_FAR_SAMPLE",
//            Loggable(
//                "SEARCH_AND_FIND", ParallelAction(
//                    race(
//                        fun(p: TelemetryPacket): Boolean {
//                            return holdPositionAction.run(p)
//                        },
//                    ),
//                )
//            ),
//        )
//    }


    fun scanForTarget(ll: Limelight, scanSpeed: Double = 10.0): LoggableAction {
        return object : LoggableAction {
            override val name: String = "SCAN_OSCILLATING"
            private var scanningRight = true
            private var initialized = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    initialized = true
                    Logging.LOG("SCAN", "Starting oscillating scan")
                }

                // Check if target is detected
                if (ll.isTagDetected && ll.trackedTag != null) {
                    Logging.LOG("SCAN", "Target found! ID: ${ll.targetId}")
                    // Hold current position
                    val holdAction = spinMotor.holdVariablePosition { spinCurrentPosition }
                    holdAction.run(p)
                    return false // Target found, stop scanning
                }

                val currentPos = spinCurrentPosition
                val minDeg = SPIN_MIN.toDouble() / ticksPerDegree
                val maxDeg = SPIN_MAX.toDouble() / ticksPerDegree

                // Reverse direction if we hit limits
                if (currentPos >= maxDeg - 1.0) {
                    scanningRight = false
                } else if (currentPos <= minDeg + 1.0) {
                    scanningRight = true
                }

                // Calculate incremental movement
                val increment = if (scanningRight) scanSpeed else -scanSpeed
                val newTarget = (currentPos + increment).coerceIn(minDeg, maxDeg)

                // Move to new position
                val targetTicks = (newTarget * ticksPerDegree).toInt().coerceIn(SPIN_MIN, SPIN_MAX)

                if (!spinMotor.lockedOut) {
                    spinMotor.spinMotor.targetPosition = targetTicks
                    spinMotor.spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    spinMotor.spinMotor.power = 0.5
                }

                Logging.LOG("SCAN", "Pos: %.1f° | Target: %.1f° | Dir: %s".format(
                    currentPos, newTarget, if (scanningRight) "→" else "←"
                ))

                return true // Keep scanning
            }
        }
    }


    override fun logState(uniqueName: String?) {
        spinMotor.logState("$uniqueName [TURRET_SPIN]")
    }

    fun lockout() {
        spinMotor.lockout()
    }

    fun unlock() {
        spinMotor.unlock()
    }


//    fun setSpinTarget(target: Double) {
//        // clamp to safe range and set motor target
//        val safe = target.coerceIn(SPIN_MIN.toDouble(), SPIN_MAX.toDouble())
//        if (safe != target) {
//            Logging.LOG("TURRET", "Requested spin target $target out of range; clamped to $safe")
//        }
//        spinTarget = safe
//        spinMotor.targetPosition = spinTarget.toInt()
//
//
//    }


//    fun interface IntProvider {
//        fun run(): Int
//    }
//
//    fun interface DoubleProvider {
//        fun run(): Double
//    }


//    fun track(): LoggableAction {
//        val holdSpinPositionAction = holdVariableSpinPosition { spinTarget.toInt() }
//        return LoggingSequential(
//            "TURRET_TRACK",
//            Loggable("TRACKING", ParallelAction(
//                race(fun(p: TelemetryPacket): Boolean {
//                    holdSpinPositionAction.run(p)
//                    return true // servo doesn't need continuous PID loop
//                })
//            ))
//        )
//    }



//    fun scanForTarget(ll: Limelight, scanSpeed: Double = 0.5, scanRange: Int = 400): LoggableAction {
//        return object : LoggableAction {
//            override val name: String = "SCAN_FOR_TARGET"
//            private var scanningRight = true
//            private var initialized = false
//            private var startPosition = 0
//
//            override fun run(p: TelemetryPacket): Boolean {
//                if (!initialized) {
//                    startPosition = spinMotor.currentPosition
//                    initialized = true
//                }
//
//                // stop scanning if target visible
//                if (ll.tx != null) return true
//
//                // compute unclamped scan target
//                val rawTarget = if (scanningRight) startPosition + scanRange else startPosition - scanRange
//
//                // clamp target to safe spin range
//                val clampedTarget = rawTarget.coerceIn(SPIN_MIN, SPIN_MAX)
//
//                // if clamp changed target, reverse direction so we don't try to force past the limit
//                if (clampedTarget != rawTarget) {
//                    scanningRight = !scanningRight
//                }
//
//                setSpinTarget(clampedTarget.toDouble())
//
//                // flip direction if we reached the scan target
//                if (scanningRight && spinCurrentPosition >= clampedTarget) scanningRight = false
//                if (!scanningRight && spinCurrentPosition <= clampedTarget) scanningRight = true
//
//                // set motor to move
//                spinMotor.targetPosition = spinTarget.toInt()
//                spinMotor.power = scanSpeed.coerceIn(0.0, 1.0)
//
//                return false // keep scanning
//            }
//        }
//    }

        //    fun scanForTarget(ll: Limelight, scanSpeed: Double = 0.005, scanRange: Double = 0.4): LoggableAction {
        //        return object : LoggableAction {
        //            override val name: String = "SCAN_FOR_TARGET"
        //            private var scanningRight = true
        //            private var initialized = false
        //            private var startPosition = 0.5 // center by default
        //
        //            override fun run(p: TelemetryPacket): Boolean {
        //                if (!initialized) {
        //                    startPosition = spinServo.position
        //                    initialized = true
        //                }
        //
        //                // stop scanning if target visible
        //                if (ll.tx != null) return true
        //
        //                // compute unclamped target position
        //                val rawTarget = if (scanningRight) startPosition + scanRange else startPosition - scanRange
        //
        //                // clamp to safe servo range (0.0..1.0)
        //                val clampedTarget = rawTarget.coerceIn(0.0, 1.0)
        //
        //                // if clamp changed the target, reverse direction
        //                if (clampedTarget != rawTarget) {
        //                    scanningRight = !scanningRight
        //                }
        //
        //                // increment servo position towards clampedTarget
        //                val nextPosition = if (scanningRight) {
        //                    (spinServo.position + scanSpeed).coerceAtMost(clampedTarget)
        //                } else {
        //                    (spinServo.position - scanSpeed).coerceAtLeast(clampedTarget)
        //                }
        //
        //                spinServo.position = nextPosition
        //
        //                // flip direction if we hit the target
        //                if (scanningRight && nextPosition >= clampedTarget) scanningRight = false
        //                if (!scanningRight && nextPosition <= clampedTarget) scanningRight = true
        //
        //                return false // keep scanning
        //            }
        //        }
        //    }


//    fun holdVariableSpinPosition(positionSpinProvider: IntProvider?): LoggableAction {
//        return object : LoggableAction {
//            override val name: String
//                get() = "$lastName HOLD_USER_DISTANCE"
//            var initialized = false
//
//            override fun run(p: TelemetryPacket): Boolean {
//                if (!initialized) {
//                    spinActionWriter.write(StringMessage("$lastName HOLD_USER_DISTANCE"))
//                    spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
//                    spinMotor.power = 1.0
//                    initialized = true
//                }
//
//                positionSpinProvider?.let {
//                    spinMotor.targetPosition = it.run()
//                }
//
//                spinPoseWriter.write(DoubleMessage(spinCurrentPosition.toDouble()))
//
//                return true
//            }
//        }
//    }

//    var lastName = ""
//    override fun logState(uniqueName: String) {
//        lastName = uniqueName
////        Logging.DEBUG("$uniqueName SPIN_POSITION", spinMotor.currentPosition)
////        Logging.DEBUG("$uniqueName SPIN_TARGET", spinMotor.targetPosition)
////        Logging.DEBUG("$uniqueName SPIN_MODE", spinMotor.mode)
////        Logging.DEBUG("$uniqueName SPIN_POWER", spinMotor.power)
////        Logging.DEBUG("$uniqueName SPIN_CURRENT", spinMotor.getCurrent(CurrentUnit.AMPS))
////        Logging.DEBUG("$uniqueName TILT_POSITION", tiltServo.position)
////        Logging.DEBUG("$uniqueName TILT_POSITION", tiltCurrentPosition)
////        Logging.DEBUG("$uniqueName TILT_TARGET", tiltTarget)
////        Logging.DEBUG("$uniqueName TILT_VOLTAGE", tiltOutput.voltage)
//    }



//    // Accept a *normalized* tilt position in [0.0, 1.0]
//    fun setTiltTargetNormalized(normalized: Double) {
//        val safe = normalized.coerceIn(TILT_MIN.toDouble(), TILT_MAX.toDouble())
//        if (safe != normalized) {
//            Logging.LOG("TURRET", "Requested tilt normalized $normalized out of range; clamped to $safe")
//        }
//        tiltTarget = safe
//        tiltServo.position = tiltTarget
//    }

//    fun setTiltTargetNormalized(normalized: Double) {
//        val safe = normalized.coerceIn(TILT_MIN, TILT_MAX)
//        if (safe != normalized) {
//            Logging.LOG("TURRET", "Requested tilt normalized $normalized out of range; clamped to $safe")
//        }
//        tiltTarget = safe
//        // Map 0.0..1.0 -> 0..3300 mV
//        val voltage = (tiltTarget * 3300).toInt()
//        tiltOutput.setAnalogOutputMode(0)       // Mode 0 = DC voltage
//        tiltOutput.setAnalogOutputVoltage(voltage)
//    }

//    fun holdVariableTiltPosition(positionTiltProvider: IntProvider?): LoggableAction {
//        return object : LoggableAction {
//            override val name: String
//                get() = "$lastName HOLD_TILT_SERVO"
//            var initialized = false
//
//            override fun run(p: TelemetryPacket): Boolean {
//                if (!initialized) {
//                    initialized = true
//                }
//                positionTiltProvider?.let {
//                    // provider returns an Int; interpret it as 0..1000 (optional) OR as already 0..1 scaled
//                    val raw = it.run().toDouble()
//                    // If provider returns encoder ticks (big numbers), map externally via setTiltTargetFromTicks.
//                    // Here we assume provider is a normalized 0..1 expressed as 0..1000 (common pattern in your TeleOp)
//                    val normalized = when {
//                        raw <= 1.0 -> raw // already 0..1
//                        raw <= 1000.0 -> (raw / 1000.0).coerceIn(0.0, 1.0)
//                        else -> (raw).coerceIn(TILT_MIN, TILT_MAX) // fallback: treat as already normalized
//                    }
//                    setTiltTargetNormalized(normalized)
//                }
//                return true
//            }
//        }
//    }

//    fun holdVariableTiltPosition(positionTiltProvider: DoubleProvider?): LoggableAction {
//        return object : LoggableAction {
//            override val name: String
//                get() = "$lastName HOLD_TILT_SERVO"
//            var initialized = false
//
//            override fun run(p: TelemetryPacket): Boolean {
//                if (!initialized) {
//                    initialized = true
//                }
//                positionTiltProvider?.let {
//                    val raw = it.run()
//                    val normalized = raw.coerceIn(TILT_MIN, TILT_MAX) // clamp to 0.0..1.0
//                    setTiltTargetNormalized(normalized)
//                }
//                return true
//            }
//        }
//    }

}
