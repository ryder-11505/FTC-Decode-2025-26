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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.TurretSpin;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpHarry extends LinearOpMode {
    public static double SlowmodeSpeed = 0.5;
    public static double SlowmodeTurning = 0.5;

    public static double TriggerMin = 0.01;

    public static double kP_Spin = Turret.ticksPerDegree; // ticksPerDeg = (encoderTicksPerRevolution / 360) * gearRatio
    public static double kI_Spin = 0.0; // ticksPerDeg ≈ 11.38
    public static double kD_Spin = 1.0; // Optii Encoders ticks/rev = 4096
                                        // Melonbotics Encoders ticks/rev = 4096
    public static double kP_Tilt = 0.17;
    public static double kI_Tilt = 0.0;
    public static double kD_Tilt = 1.0;

    public static double deg = 0.0;

    private double spinIntegral = 0;
    private double spinPrevError = 0;

    private double tiltIntegral = 0;
    private double tiltPrevError = 0;

    public int targetId = 24; // default tag ID

    private LoggableAction scanningAction = null;
    private LoggableAction trackingAction = null;
    private boolean isScanning = false;



    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Limelight ll = new Limelight(hardwareMap, "limelight");
//        Turret turret = new Turret(hardwareMap);
        TurretSimple spinSimple = new TurretSimple(hardwareMap);

//        LLResult result = visionDetection.limelight.getLatestResult();
//        LLResultTypes.FiducialResult fr = (LLResultTypes.FiducialResult) result.getFiducialResults();
//
//        int tagId = fr.getFiducialId();

        Pose2d pose = driveBase.localizer.getPose();


//        turret.unlock();

        Button fieldMode = new Button();
        Button slowMode = new Button();

        double X = pose.position.x;

        double Y = pose.position.y;

        double H = Math.toDegrees(pose.heading.toDouble());

        double Dr = Math.sqrt(Math.pow((72 - Y), 2) + Math.pow((72 + X), 2));

        double Db = Math.sqrt(Math.pow((-72 - Y), 2) + Math.pow((72 + X), 2));

        double Hr = (- Math.atan(((X + 72) / (Y + 72))) + H);

        double Hb = Math.atan(((X + 72) / (Y + 72)) - 180 + H);

        double Hm = Math.asin(((X + 72) / (Y + 72)) + H);

        double a = (7.5 / 97) * (53);

        double b = 22 - a;

        double RSr = (7.5 / 97) * (Dr + 19) + b;

        double RSb = (7.5 / 97) * (Db + 19) + b;

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket p;


        PoseStorage.splitControls = true;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            visionDetection.update(driveBase.localizer, p);
            spinSimple.resetEncoder();
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

        spinSimple.resetEncoder();

        FinishingState finishState = FinishingState.Outtake;



        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(p);


            if (gamepad1.right_bumper) {
                spinSimple.track(116);
            }

            if (gamepad1.rightBumperWasReleased()) {
                spinSimple.track(0);
            }

            if (gamepad1.left_bumper) {
                spinSimple.track(-116);
            }

            if (gamepad1.leftBumperWasReleased()) {
                spinSimple.track(0);
            }



//            if (gamepad1.right_bumper && PoseStorage.isRedAlliance) {
//                spinSimple.track(Hr);
//            } else if (gamepad1.right_bumper && !PoseStorage.isRedAlliance) {
//                spinSimple.track(Hb);
//            }
//
//            if (gamepad1.rightBumperWasReleased()) {
//                spinSimple.track(0);
//            }
//
//            if (gamepad1.x && PoseStorage.isRedAlliance) {
//                outtake.setPower(RSr);
//                Logging.LOG("x");
//            } else if (gamepad1.x && !PoseStorage.isRedAlliance) {
//                outtake.setPower(RSb);
//                Logging.LOG("x");
//            }
//
//            if (gamepad1.xWasReleased()) {
//                outtake.stopShoot();
//            }


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

//            slowMode.update(gamepad1.y);
//            fieldMode.update(gamepad1.x);


            driveBase.update(p);


            if (gamepad1.x) {
//                outtake.setPower(RS);
                outtake.shootShort();
                Logging.LOG("x");
            }

            if (gamepad1.xWasReleased()) {
                outtake.stopShoot();
            }

            if (gamepad1.y) {
                outtake.shootLong();
                Logging.LOG("y");
            }

            if (gamepad1.yWasReleased()) {
                outtake.stopShoot();
            }

            if (gamepad1.a) {
                intake.intake();
                Logging.LOG("a");
            }

            if (gamepad1.aWasReleased()) {
                intake.stopIntake();
            }


            if (gamepad1.b) {
                outtake.shootSuperShort();
                Logging.LOG("b");
            }

            if (gamepad1.bWasReleased()) {
                outtake.stopShoot();
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

            Logging.LOG("SCANNING_MODE", isScanning);
            if (isScanning) {
                Logging.LOG("SCAN_STATE", scanningAction != null ? "SCANNING" : "TRACKING");
            }
            Logging.LOG("shooter velocity (rad/s)", outtake.getMotor().getVelocity(AngleUnit.RADIANS));
            Logging.LOG("shooter velocity (ticks/s)", outtake.getMotor().getVelocity());
            Logging.LOG("shooter power", outtake.getMotor().getPower());
            Logging.LOG("Spin Target", spinSimple.getSpinTargetPos());
            Logging.LOG("Spin Pos", spinSimple.getSpinCurrentPosition());
            Logging.LOG("Spin Status", spinSimple.getSpinMotor().isBusy());
            Logging.LOG("Spin Power", spinSimple.getSpinMotor().getPower());
            Logging.LOG("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
//            Logging.LOG("SPLIT", PoseStorage.splitControls);
            Logging.LOG("FIELD_MODE", fieldMode.val);
            Logging.LOG("SLOW_MODE", slowMode.val);
            Logging.LOG("HALLUCINATING", PoseStorage.shouldHallucinate);
            Logging.LOG("LL_TAG_DETECTED", ll.isTagDetected());
//            Logging.LOG("LL_TX", ll.getTx());
//            Logging.LOG("LL_TY", ll.getTy());
//            if (ll.isTagDetected()) {
//                Logging.LOG("LL_DETECTED_ID", ll.getDetectedId());
//            }


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

    enum Motif {
        PPG,
        PGP,
        GPP,
        None
    }
}
