package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretFunctions.DoubleProvider
import org.firstinspires.ftc.teamcode.messages.DoubleMessage
import org.firstinspires.ftc.teamcode.messages.StringMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.max
import kotlin.math.min


@Config
class Turret(hardwareMap: HardwareMap) : StateLoggable {

    companion object PARAMS {
        @JvmField // Spin Target
        var spinTarget: Double = 0.0

        @JvmField
        var tiltTarget: Double = 0.0

        // REV Through Bore Encoder = 8192 CPR, 4:1 gear ratio (20T:80T)
        private const val encoderCPR = 8192.0
        private const val gearRatio = 20.0
        private const val ticksPerRev = encoderCPR * gearRatio
        @JvmField
        val ticksPerDegree: Double = ticksPerRev / 360.0  // ~91.0 ticks/deg

        // Mechanical limits (adjust to your turret’s safe travel)
        // Example: ±90° range
        @JvmField
        val SPIN_MIN = -(90 * ticksPerDegree).toInt()   // ≈ -8192
        @JvmField
        val SPIN_MAX =  (90 * ticksPerDegree).toInt()   // ≈ +8192

        // Tilt limits (servo normalized range)
        const val TILT_MIN = 0.0
        const val TILT_MAX = 1.0

        @JvmField var kP_Tilt = 0.17
        @JvmField var kI_Tilt = 0.0
        @JvmField var kD_Tilt = 0.1
    }


    val spinMotor = hardwareMap.get(DcMotorEx::class.java, "spin")
//    val tiltMotor = hardwareMap.get(DcMotorEx::class.java, "tilt")
//    val spinServo = hardwareMap.get(Servo::class.java, "spin") // servo
//    val spinOutput = hardwareMap.get(AnalogInput::class.java, "spinOutput")
    val tiltServo = hardwareMap.get(Servo::class.java, "tilt") // servo
//    val tiltOutput = hardwareMap.get(AnalogInput::class.java, "tiltOutput")

//

    val spinCurrentPosition get() = spinMotor.currentPosition
//    val tiltCurrentPosition get() = tiltMotor.currentPosition
//    val spinCurrentPosition get() = spinTarget
    val tiltCurrentPosition get() = tiltTarget // servo doesn't have encoder

//    // AnalogInput gives volts in range ~0-5 (5V)
//    val tiltCurrentPosition: Double
//        get() {
//            val volts = tiltOutput.voltage
//            // Map 0–5 V to 0.0–1.0 normalized range
//            return (volts / 5).coerceIn(TILT_MIN, TILT_MAX)
//        }

//    private var spinTarget = 0.5  // normalized target
//    private var spinIntegral = 0.0
    private var tiltTarget = 0.5  // normalized target
    private var tiltIntegral = 0.0
    private var lastError = 0.0
    private val timer = ElapsedTime()


//    private fun initMotor() {
//        spinMotor.targetPosition = 0
//        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
//        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        spinMotor.power = 0.1
//        spinMotor.targetPositionTolerance = (tolerance * 0.1).toInt()
//        tiltMotor.targetPosition = 0
//        tiltMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
//        tiltMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        tiltMotor.power = 0.1
//        tiltMotor.targetPositionTolerance = (tolerance * 0.1).toInt()
//    }

    private fun initMotor() {
        spinMotor.targetPosition = 0
        spinMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.power = 0.1
    }

//    init {
//        initMotor()
//        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        tiltMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
//        tiltMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
//    }

    init {
        initMotor()
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        // tiltTarget should already be in 0.0..1.0 range
//        spinServo.position = spinTarget.coerceIn(0.0, 1.0)
        tiltServo.position = tiltTarget.coerceIn(TILT_MIN, TILT_MAX)
    }

    private val spinActionWriter = DownsampledWriter("SPIN_ACTION", 50_000_000)
    private val spinPoseWriter = DownsampledWriter("SPIN_POSITION", 50_000_000)
    private val tiltActionWriter = DownsampledWriter("TILT_ACTION", 50_000_000)
    private val tiltPoseWriter = DownsampledWriter("TILT_POSITION", 50_000_000)



    // --- PID Correction ---
    // Apply proportional correction so the turret "tracks" the center
    fun aimTiltFromTyPID(ty: Double) {
        val error = ty / 20.0  // scale FOV ±20° -> ±1.0
        val correction = kP_Tilt * error
        val newTarget = (tiltTarget + correction).coerceIn(TILT_MIN, TILT_MAX)
        setTiltTargetNormalized(newTarget)
    }

    fun setSpinTarget(target: Double) {
        // clamp to safe range and set motor target
        val safe = target.coerceIn(SPIN_MIN.toDouble(), SPIN_MAX.toDouble())
        if (safe != target) {
            Logging.LOG("TURRET", "Requested spin target $target out of range; clamped to $safe")
        }
        spinTarget = safe
        spinMotor.targetPosition = spinTarget.toInt()
    }


//    fun setTiltTarget(targetTicks: Double) {
//        // Clamp target to old tick range
//        val clampedTicks = targetTicks.coerceIn(TILT_MIN.toDouble(), TILT_MAX.toDouble())
//        // Map to servo 0.0..1.0
//        val servoPosition = (clampedTicks - TILT_MIN) / (TILT_MAX - TILT_MIN)
//        tiltTarget = servoPosition.coerceIn(0.0, 1.0)
//        tiltServo.position = tiltTarget
//    }

    // Accept a *normalized* tilt position in [0.0, 1.0]
    fun setTiltTargetNormalized(normalized: Double) {
        val safe = normalized.coerceIn(TILT_MIN.toDouble(), TILT_MAX.toDouble())
        if (safe != normalized) {
            Logging.LOG("TURRET", "Requested tilt normalized $normalized out of range; clamped to $safe")
        }
        tiltTarget = safe
        tiltServo.position = tiltTarget
    }

//

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


    fun interface IntProvider {
        fun run(): Int
    }

    fun interface DoubleProvider {
        fun run(): Double
    }




    fun track(): LoggableAction {
        val holdSpinPositionAction = holdVariableSpinPosition { spinTarget.toInt() }
        return LoggingSequential(
            "TURRET_TRACK",
            Loggable("TRACKING", ParallelAction(
                race(fun(p: TelemetryPacket): Boolean {
                    holdSpinPositionAction.run(p)
                    return true // servo doesn't need continuous PID loop
                })
            ))
        )
    }

//

    fun scanForTarget(ll: Limelight, scanSpeed: Double = 0.5, scanRange: Int = 400): LoggableAction {
        return object : LoggableAction {
            override val name: String = "SCAN_FOR_TARGET"
            private var scanningRight = true
            private var initialized = false
            private var startPosition = 0

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    startPosition = spinMotor.currentPosition
                    initialized = true
                }

                // stop scanning if target visible
                if (ll.tx != null) return true

                // compute unclamped scan target
                val rawTarget = if (scanningRight) startPosition + scanRange else startPosition - scanRange

                // clamp target to safe spin range
                val clampedTarget = rawTarget.coerceIn(SPIN_MIN, SPIN_MAX)

                // if clamp changed target, reverse direction so we don't try to force past the limit
                if (clampedTarget != rawTarget) {
                    scanningRight = !scanningRight
                }

                setSpinTarget(clampedTarget.toDouble())

                // flip direction if we reached the scan target
                if (scanningRight && spinCurrentPosition >= clampedTarget) scanningRight = false
                if (!scanningRight && spinCurrentPosition <= clampedTarget) scanningRight = true

                // set motor to move
                spinMotor.targetPosition = spinTarget.toInt()
                spinMotor.power = scanSpeed.coerceIn(0.0, 1.0)

                return false // keep scanning
            }
        }
    }

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


    fun holdVariableSpinPosition(positionSpinProvider: IntProvider?): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName HOLD_USER_DISTANCE"
            var initialized = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    spinActionWriter.write(StringMessage("$lastName HOLD_USER_DISTANCE"))
                    spinMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    spinMotor.power = 1.0
                    initialized = true
                }

                positionSpinProvider?.let {
                    spinMotor.targetPosition = it.run()
                }

                spinPoseWriter.write(DoubleMessage(spinCurrentPosition.toDouble()))

                return true
            }
        }
    }


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

    fun holdVariableTiltPosition(positionTiltProvider: DoubleProvider?): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName HOLD_TILT_SERVO"
            var initialized = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    initialized = true
                }
                positionTiltProvider?.let {
                    val raw = it.run()
                    val normalized = raw.coerceIn(TILT_MIN, TILT_MAX) // clamp to 0.0..1.0
                    setTiltTargetNormalized(normalized)
                }
                return true
            }
        }
    }

//    fun holdVariableSpinPosition(positionTiltProvider: DoubleProvider?): LoggableAction {
//        return object : LoggableAction {
//            override val name: String
//                get() = "$lastName HOLD_SPIN_SERVO"
//            var initialized = false
//
//            override fun run(p: TelemetryPacket): Boolean {
//                if (!initialized) {
//                    initialized = true
//                }
//                positionTiltProvider?.let {
//                    val raw = it.run()
//                    val normalized = raw.coerceIn(SPIN_MIN, SPIN_MAX) // clamp to 0.0..1.0
//                    setSpinTargetNormalized(normalized)
//                }
//                return true
//            }
//        }
//    }

    var lastName = ""
    override fun logState(uniqueName: String) {
        lastName = uniqueName
        Logging.DEBUG("$uniqueName SPIN_POSITION", spinMotor.currentPosition)
        Logging.DEBUG("$uniqueName SPIN_TARGET", spinMotor.targetPosition)
        Logging.DEBUG("$uniqueName SPIN_MODE", spinMotor.mode)
        Logging.DEBUG("$uniqueName SPIN_POWER", spinMotor.power)
        Logging.DEBUG("$uniqueName SPIN_CURRENT", spinMotor.getCurrent(CurrentUnit.AMPS))
//        Logging.DEBUG("$uniqueName TILT_POSITION", tiltMotor.currentPosition)
//        Logging.DEBUG("$uniqueName TILT_TARGET", tiltMotor.targetPosition)
//        Logging.DEBUG("$uniqueName TILT_MODE", tiltMotor.mode)
//        Logging.DEBUG("$uniqueName TILT_POWER", tiltMotor.power)
//        Logging.DEBUG("$uniqueName TILT_CURRENT", tiltMotor.getCurrent(CurrentUnit.AMPS))
        Logging.DEBUG("$uniqueName TILT_POSITION", tiltServo.position)
        Logging.DEBUG("$uniqueName TILT_POSITION", tiltCurrentPosition)
        Logging.DEBUG("$uniqueName TILT_TARGET", tiltTarget)
//        Logging.DEBUG("$uniqueName TILT_VOLTAGE", tiltOutput.voltage)
//        Logging.DEBUG("$uniqueName SPIN_POSITION", spinServo.position)
//        Logging.DEBUG("$uniqueName SPIN_POSITION", spinCurrentPosition)
//        Logging.DEBUG("$uniqueName SPIN_TARGET", spinTarget)
//        Logging.DEBUG("$uniqueName SPIN _VOLTAGE", spinTarget * 3300)
    }

}
