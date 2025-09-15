package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpHarry extends LinearOpMode {
    public static double SlowmodeSpeed = 0.5;
    public static double SlowmodeTurning = 0.5;


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);


        Button fieldMode = new Button();
        Button slowMode = new Button();



        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket p;


        PoseStorage.splitControls = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);

            Logging.update();

            FtcDashboard.getInstance().sendTelemetryPacket(p);

            if (gamepad2.back) {
                PoseStorage.splitControls = true;
            } else if (gamepad1.back) {
                PoseStorage.splitControls = false;
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



        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();


            if (gamepad2.back) {
                PoseStorage.splitControls = true;
            } else if (gamepad1.back) {
                PoseStorage.splitControls = false;
            }

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;


//            if (PoseStorage.splitControls) {
//                slowMode.update(gamepad1.left_bumper);
//                fieldMode.update(gamepad1.x);
//
//                if (gamepad2.dpad_right) {
//                    PoseStorage.isRedAlliance = true;
//                } else if (gamepad2.dpad_left) {
//                    PoseStorage.isRedAlliance = false;
//                }
//            } else {
//                slowMode.update(gamepad1.y);
//                fieldMode.update(gamepad1.x);
//
//                if (gamepad1.dpad_right) {
//                    PoseStorage.isRedAlliance = true;
//                } else if (gamepad1.dpad_left) {
//                    PoseStorage.isRedAlliance = false;
//                }
//            }

            driveBase.update(p);


            if (gamepad1.xWasPressed()){
                outtake.shoot();
            }

            if (gamepad1.xWasReleased()){
                outtake.stopShoot();
            }

            if (gamepad1.aWasPressed()){
                intake.intake();
            }

            if (gamepad1.aWasReleased()){
                intake.stopIntake();
            }

            if (gamepad1.rightBumperWasPressed()){
                intake.intake();
            }

            if (gamepad1.rightBumperWasReleased()){
                intake.stopIntake();
            }

            if (gamepad1.bWasPressed()){
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
            Logging.LOG("SPLIT", PoseStorage.splitControls);
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
