package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpHarry extends LinearOpMode {
    public static double SlowmodeSpeed = 0.5;
    public static double SlowmodeTurning = 0.5;

    public static double kP_Spin = Turret.ticksPerDegree; // ticksPerDeg = (encoderTicksPerRevolution / 360) * gearRatio
    public static double kI_Spin = 0.0; // ticksPerDeg ≈ 11.38
    public static double kD_Spin = 1.0; // Optii Encoders ticks/rev = 4096
                                        // Melonbotics Encoders ticks/rev = 4096
    public static double kP_Tilt = 0.17;
    public static double kI_Tilt = 0.0;
    public static double kD_Tilt = 1.0;

    private double spinIntegral = 0;
    private double spinPrevError = 0;

    private double tiltIntegral = 0;
    private double tiltPrevError = 0;

    private int targetId = 24; // default tag ID




    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Limelight ll = new Limelight(hardwareMap, "limelight");
        Turret turret = new Turret(hardwareMap);

        Button fieldMode = new Button();
        Button slowMode = new Button();

        double g = 9.81; // Gravity
        double d = ll.getHorizontalDistance(); // Horizontal disatnce
        double h = 0.0; // Target height - Shooter height
        double r = 0.0; // Flywheel radius
        double θ = Math.toRadians(ll.getTy()); // Launch angle

        double v = Math.sqrt((g * (d * d)) / (2 * (Math.cos(θ) * Math.cos(θ)) * (d * Math.tan(θ) + h))); // projectile muzzle velocity m/s

        double RPM = (60 * v) / (2 * Math.PI * r); // Flywheel RPM


        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket p;


        PoseStorage.splitControls = true;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            ll.setTargetIdProvider(() -> targetId);

            Logging.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);

//            if (gamepad2.back) {
//                PoseStorage.splitControls = true;
//            } else if (gamepad1.back) {
//                PoseStorage.splitControls = false;
//            }

            if (gamepad2.dpad_right) {
                PoseStorage.isRedAlliance = true; // Tag ID 24
                targetId = 24;
            } else if (gamepad2.dpad_left) {
                PoseStorage.isRedAlliance = false; // Tag ID 20
                targetId = 20;
            }
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        LoggableAction finishingAction = new Loggable("INIT", new ParallelAction(

        ));


        FinishingState finishState = FinishingState.Outtake;

        LoggableAction scanningAction = turret.scanForTarget(ll, 0.5, 400);


        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();
            if (scanningAction.run(p)) break; // Target found
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            

            // Run turret tracking
            turret.track().run(p);
//
//            // --- Tilt PID ---
//            tiltIntegral += ll.getTy();
//            double tiltDerivative = ll.getTy() - tiltPrevError;
//            double tiltOutput = (kP_Tilt * ll.getTy())
//                    + (kI_Tilt * tiltIntegral)
//                    + (kD_Tilt * tiltDerivative);
//            tiltPrevError = ll.getTy();
//
//            // Send outputs to turret (scale/clamp if needed)
//            turret.setTiltTarget(Turret.tiltTarget + tiltOutput);

//            if (Math.abs(ll.getTy()) < 0.5) tiltIntegral = 0;

            // --- Spin PID ---
            spinIntegral += ll.getTx();
            double spinDerivative = ll.getTx() - spinPrevError;
            double spinOutput = (kP_Spin * ll.getTx())
                    + (kI_Spin * spinIntegral)
                    + (kD_Spin * spinDerivative);
            spinPrevError = ll.getTx();

            // Apply spin PID to motor
            turret.setSpinTarget(Turret.spinTarget + spinOutput);


            // --- Tilt correction (servo version) ---
            double tiltCorrection = kP_Tilt * ll.getTy() / 20.0; // ±20° vertical FOV
            double newTiltTarget = Turret.tiltTarget + tiltCorrection;
            turret.setTiltTargetNormalized(newTiltTarget);

            turret.aimTiltFromTyPID(ll.getTy());

//

//            // Reset spin integral near zero
//            if (Math.abs(ll.getTx()) < 0.5) spinIntegral = 0;

            // Show current values on DS
            telemetry.addData("Spin Target", Turret.spinTarget);
            telemetry.addData("Tilt Target", Turret.tiltTarget);
            telemetry.addData("Spin Pos", turret.getSpinCurrentPosition());
            telemetry.addData("Tilt Pos", turret.getTiltCurrentPosition());
            telemetry.update();

//            if (gamepad2.back) {
//                PoseStorage.splitControls = true;
//            } else if (gamepad1.back) {
//                PoseStorage.splitControls = false;
//            }

            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;



            if (gamepad2.dpad_right) {
                PoseStorage.isRedAlliance = true; // Tag ID 24
                targetId = 24;
            } else if (gamepad2.dpad_left) {
                PoseStorage.isRedAlliance = false; // Tag ID 20
                targetId = 20;
            }

            slowMode.update(gamepad1.y);
            fieldMode.update(gamepad1.x);


            driveBase.update(p);


            if (gamepad1.x){
                outtake.setPower(RPM);
            }

            if (gamepad1.xWasReleased()){
                outtake.stopShoot();
            }

            if (gamepad1.a){
                intake.intake();
            }

            if (gamepad1.aWasReleased()){
                intake.stopIntake();
            }

            if (gamepad1.right_bumper){
                intake.intake();
            }

            if (gamepad1.rightBumperWasReleased()){
                intake.stopIntake();
            }

            if (gamepad1.b){
                intake.outake();
            }

            if (gamepad1.bWasReleased()){
                intake.stopIntake();
            }



            Pose2d poseEstimate = driveBase.localizer.getPose();
            input = input.times(slowMode.val ? SlowmodeSpeed : 1);

            if (fieldMode.val) {
                input = poseEstimate.heading.inverse().plus((PoseStorage.isRedAlliance ? 1 : -1) * Math.PI / 2).times(input);
            }

            PoseVelocity2d drivePower = new PoseVelocity2d(
                    input,
                    -gamepad1.right_stick_x * (slowMode.val ? SlowmodeTurning : 1)
            );

            Logging.DEBUG("X Input", input.x);
            Logging.DEBUG("Y Input", input.y);

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            driveBase.setDrivePowers(drivePower);




            Logging.LOG("FINISH_STATE", finishState);
            if (finishingAction != null) {
                Logging.DEBUG("FINISH_ACTION", finishingAction.getName());

                if (!finishingAction.run(p)) {
                    Logging.LOG("FINISH_ACTION_FINISHED");
                    finishingAction = null;
                    finishState = FinishingState.None;
                }
            }





            Logging.LOG("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
//            Logging.LOG("SPLIT", PoseStorage.splitControls);
            Logging.LOG("FIELD_MODE", fieldMode.val);
            Logging.LOG("SLOW_MODE", slowMode.val);
            Logging.LOG("HALLUCINATING", PoseStorage.shouldHallucinate);

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[TELEOP]");

            Logging.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }



    enum FinishingState {
        Intake,
        Outtake,
        None
    }
}
