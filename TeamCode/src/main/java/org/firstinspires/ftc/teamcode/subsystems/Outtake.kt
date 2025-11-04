package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple.PARAMS.ticksPerDegree

@Config
class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField
        var speedSuperShort = ((21.5 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speedShort = ((22.0 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/62

        @JvmField
        var speedLong = ((29.5 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speed = 1300.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var open = 0.5125

        @JvmField
        var closed = 0.25

    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "outL")
    val servo = hardwareMap.get(Servo::class.java, "servo")
    val servo2 = hardwareMap.get(Servo::class.java, "servo2")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.setVelocityPIDFCoefficients(25.0,0.0,0.0,20.0)
//        motor.setVelocityPIDFCoefficients(1.17025,0.117025,0.0,11.7025)
        servo.position = closed
        servo2.position = 1 - closed
    }


    fun open() {
        servo.position = open
        servo2.position = 1 - open
    }

    fun close() {
        servo.position = closed
        servo2.position = 1 - closed
    }

    fun shootSuperShort3() {
        motor.setVelocity(speedSuperShort)
//        if (IntRange(-20, 20).contains(((speedSuperShort).toInt()) - ((motor.velocity).toInt()))){
//            servo.position = open
//        } else {
//            servo.position = closed
//        }
        if ((motor.velocity) < (speedSuperShort)) {
            servo.position = closed
            servo2.position = 1 - closed
        } else {
            servo.position = open
            servo2.position = 1 - open
        }

    }

    fun shootShort3() {
        motor.setVelocity(speedShort)
        if (IntRange(-20, 20).contains(((speedShort).toInt()) - ((motor.velocity).toInt()))){
            servo.position = open
            servo2.position = 1 - open
        } else {
            servo.position = closed
            servo2.position = 1 - closed
        }
    }

    fun shootLong3() {
        motor.setVelocity(speedLong)
        if (IntRange(-20, 20).contains(((speedLong).toInt()) - ((motor.velocity).toInt()))){
            servo.position = open
            servo2.position = 1 - open
        } else {
            servo.position = closed
            servo2.position = 1 - closed
        }
    }

    fun setPower3(RS: Double) {
        val varSpeed = ((RS * 10.0) / (2.0 * Math.PI)) * 28.0
        motor.setVelocity(varSpeed)
        if (motor.velocity == varSpeed) {
            servo.position = open
            servo2.position = 1 - open
        } else {
            servo.position = closed
            servo2.position = 1 - closed
        }
    }

    fun shootSuperShort() {
        motor.setVelocity(speed)
        servo.position = open
    }

    fun shootShort() {
        motor.setVelocity(speedShort)
        servo.position = open
    }

    fun shootLong() {
        motor.setVelocity(speedLong)
        servo.position = open
    }

    fun stopShoot() {
        motor.setVelocity(0.0)
        servo.position = closed
        servo2.position = 1 - closed
    }

    override fun logState(uniqueName: String?) {
        Logging.DEBUG("$uniqueName SHOOTER_POWER", motor.power)
        Logging.DEBUG("$uniqueName SHOOTER_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
    }
}
