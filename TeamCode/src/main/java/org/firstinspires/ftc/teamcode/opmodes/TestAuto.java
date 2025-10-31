package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOneForDoubleSpecimen;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretSimple;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;
import java.util.logging.LoggingMXBean;

@Autonomous(name = "Test Auto")
@Config
public class TestAuto extends LinearOpMode {
    public static TrajectoryActionBuilder addParts(TrajectoryActionBuilder traj, AutoPart[] parts) {
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value+0.00001);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    public static TrajectoryActionBuilder addActionAfterFirst(TrajectoryActionBuilder traj, AutoPart[] parts, Action action) {
        boolean addedFirst = false;
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }

            if (!addedFirst) {
                addedFirst = true;
                traj = traj.afterTime(0.1, action);
            }
        }
        return traj;
    }

    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Limelight ll = new Limelight(hardwareMap, "limelight");
//        Turret turret = new Turret(hardwareMap);
        TurretSimple spinSimple = new TurretSimple(hardwareMap);


        LLResult result = visionDetection.limelight.getLatestResult();
        LLResultTypes.FiducialResult fr = (LLResultTypes.FiducialResult) result.getFiducialResults();

        int tagId = fr.getFiducialId();

        Motif motif = Motif.None;

        if (result.isValid()){
            fr.getFiducialId();
            if (fr.getFiducialId() == 21){
                motif = Motif.GPP;
            } else if (fr.getFiducialId() == 22){
                motif = Motif.PGP;
            } else if (fr.getFiducialId() == 23){
                motif = Motif.PPG;
            }
            switch (motif) {
                case GPP:
                    // Go to corresponding group of artifacts on field
                    break;
                case PGP:
                    // Go to corresponding group of artifacts on field
                    break;
                case PPG:
                    // Go to corresponding group of artifacts on field
                    break;
            }
        } else {
            Logging.LOG("No Target Found");
        }

        Pose2d pose = driveBase.localizer.getPose();

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


        Action initAction = new ParallelAction(

        );

//        Run initialisation tasks
        visionDetection.limelight.start();
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }



        boolean innerPosition = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);

            if (gamepad1.a) innerPosition = true;
            else if (gamepad1.y) innerPosition = false;

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false;

            }

            Logging.LOG("ALLIANCE", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("POSITION", innerPosition ? "INNER" : "OUTER");

            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        Map map = innerPosition ? new InsideOne() : new OutsideOneForDoubleSpecimen();

        TrajectoryActionBuilder builder = driveBase.allianceActionBuilder(map.getStartPosition());

        driveBase.localizer.setPose(PoseStorage.isRedAlliance
                ? map.getStartPosition()
                : new Pose2d(
                -map.getStartPosition().position.x,
                -map.getStartPosition().position.y,
                map.getStartPosition().heading.plus(Math.PI).toDouble()));


        // Scan Motif
        builder = builder.strafeTo(map.getSpecimenPosition().position)
                .stopAndAdd(
                        new SequentialAction(
                                new Action() {
                                    @Override
                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                        outtake.shootSuperShort();
                                        return false;
                                    }
                                }
                        )
                ).strafeTo(new Vector2d(0, -52));
        // Strafe to relevant set of artifacts on field
        builder = addParts(builder, map.getParkParts());

        // Intake artifacts
        builder = builder.strafeTo(map.getSpecimenPosition().position)
                .stopAndAdd(
                        new SequentialAction(
//                                outtake.raiseSpecimen(true),
//                                new Timeout(
//                                        outtake.ensureSpecimenPlaced(), 3
//                                ),
//                                outtake.safeAutoReturnSpecimen()
                        )
                ).strafeTo(new Vector2d(0, -52));
        // Strafe to shooting position
        builder = addParts(builder, map.getParkParts());

        // Lock onto apriltag relevant to alliance and shoot the artifacts
        builder = builder.strafeTo(map.getCollectPosition().position)
                .stopAndAdd(
                        new SequentialAction(
//                                outtake.specimenReady(true),
//                                new SleepAction(0.5),
//                                outtake.grabber(false),
//                                outtake.getLift().gotoDistance(10.0),
//                                outtake.raiseSpecimen(true)
                        )
                ).strafeTo(new Vector2d(0, -52));
        // Strafe to park position
        builder = addParts(builder, map.getDepositParts());


        Action autonomous = builder.build();

        while (opModeIsActive() && !isStopRequested() && autonomous.run(p)) {
            p = new TelemetryPacket();

            driveBase.update(p);
//            visionDetection.update(driveBase.localizer, p);
            PoseStorage.currentPose = driveBase.localizer.getPose();

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[AUTO]");
            intake.logState("[AUTO]");
            outtake.logState("[AUTO]");
            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }

    }

    enum Motif {
        PPG,
        PGP,
        GPP,
        None
    }
}