package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOneForDoubleSpecimen;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

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
        Limelight ll = new Limelight(hardwareMap, "limelight");
        Turret turret = new Turret(hardwareMap);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Action startllAction = new Action() {
            @Override
            public boolean run(TelemetryPacket p) {
                ll.start(); // call start once
                return true; // immediately done
            }
        };

        Action initAction = new ParallelAction(
//                turret.scanForTarget(ll, 0.5, 400),
                startllAction,
                new SequentialAction(

                )
        );

//        Run initialisation tasks
        turret.unlock();
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }


        turret.lockout();

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

        turret.unlock();

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
//                                outtake.raiseSpecimen(true),
//                                new Timeout(
//                                        outtake.ensureSpecimenPlaced(), 3
//                                ),
//                                outtake.safeAutoReturnSpecimen()
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
}