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
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple.PARAMS
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple.PARAMS.ticksPerDegree

@Config
class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField
        var speedSuperShort = ((21.5 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speedShort = ((22.0 * 10.0) / (2.0 * Math.PI)) * 28.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/62

        @JvmField
        var speedLong = 990.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var speed = 1300.0 // 1.0 power ≈ 628 rad/s ∴ rad/s = 628(power) or rad/s/628 = power

        @JvmField
        var open = 0.5125

        @JvmField
        var closed = 0.25


    }

    val motor = hardwareMap.get(DcMotorEx::class.java, "outL")
    val servo = hardwareMap.get(Servo::class.java, "servo")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.setVelocityPIDFCoefficients(34.0,0.0,0.0,20.0)
//        motor.setVelocityPIDFCoefficients(1.17025,0.117025,0.0,11.7025)
        servo.position = closed
    }


    fun open() {
        servo.position = open
    }

    fun close() {
        servo.position = closed
    }

    fun shootSuperShort() {
        motor.setVelocity(speedSuperShort)
        if ((motor.velocity) < (speedSuperShort)) {
            servo.position = closed
        } else {
            servo.position = open
        }

    }

    fun shootShort() {
        motor.setVelocity(speedShort)
        if ((motor.velocity) < (speedShort)) {
            servo.position = closed
        } else {
            servo.position = open
        }
    }

    fun shootLong() {
        motor.setVelocity(speedLong)
        if ((motor.velocity) < (speedLong)) {
            servo.position = closed
        } else {
            servo.position = open
        }
    }

    fun setPower(RS: Double): Boolean {
        motor.setVelocity(RS)
        if ((motor.velocity) < (RS)) {
            servo.position = closed
            return false
        } else {
            servo.position = open
            return true
        }
    }

    fun stopShoot() {
        motor.setVelocity(0.0)
        servo.position = closed
    }



    override fun logState(uniqueName: String?) {
        Logging.DEBUG("$uniqueName SHOOTER_POWER", motor.power)
        Logging.DEBUG("$uniqueName SHOOTER_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
    }
}
