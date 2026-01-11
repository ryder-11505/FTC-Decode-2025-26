package com.example.meepmeepvis;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.noahbres.meepmeep.roadrunner.entity.TrajectoryActionStub;
import com.noahbres.meepmeep.roadrunner.entity.TurnActionStub;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOneBlue;
import com.userjhansen.automap.Maps.InsideOneRed;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOneBlue;
import com.userjhansen.automap.Maps.OutsideOneRed;

public class MeepMeepVis {

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
                    traj = traj.waitSeconds(2);
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    public static Action buildTrajectorySequence(DriveShim drive, Map map, boolean isRed) {
        TrajectoryActionBuilder baseTrajBuilder = drive.actionBuilder(
                new Pose2d(0,0,0)
        );

        TrajectoryActionBuilder traj = new TrajectoryActionBuilder(
                TurnActionStub::new,
                TrajectoryActionStub::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                map.getStartPosition(), 0.0,
                baseTrajBuilder.getBaseTurnConstraints(), baseTrajBuilder.getBaseVelConstraint(), baseTrajBuilder.getBaseAccelConstraint(),
                isRed ? pose -> pose
                        : pose -> new Pose2dDual<>(
                                pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.plus(Math.PI))

        );

        traj = addParts(traj, AutoPart.makeFullAutoList(map));

        return traj.build();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        boolean isRed = false;
        boolean innerPosition = false;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.0, 18.0)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(isRed ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                .build();


        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.0, 18.0)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(!isRed ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                .build();

        Map mapR = innerPosition ? new InsideOneRed() : new OutsideOneRed();

        Map mapB = innerPosition ? new InsideOneBlue() : new OutsideOneBlue();

        Action blueTraj = bot.getDrive().actionBuilder(mapB.getStartPosition())
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(-64.0, -34.5))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(-11.75, -27.0))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(-11.75, -50.0))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(-64.0, -34.5))
                .build();


        Action redTraj = bot.getDrive().actionBuilder(mapR.getStartPosition())
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(-64.0, 34.5))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(-11.45, 27.0))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(-11.45, 50.0))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(-64.0, 34.5))
                .build();

        bot.runAction(isRed ? redTraj : blueTraj);

        Map mapR2 = !innerPosition ? new InsideOneRed() : new OutsideOneRed();

        Map mapB2 = !innerPosition ? new InsideOneBlue() : new OutsideOneBlue();

        Action blueTraj2 = bot2.getDrive().actionBuilder(mapB2.getStartPosition())
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(60.0, -17.5))
                .strafeToLinearHeading(new Vector2d(35.95, -27.0), new Rotation2d(Math.toRadians(0.0), Math.toRadians(270.0)))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(35.95, -50.0))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(60.0, -17.5))
                .build();


        Action redTraj2 = bot2.getDrive().actionBuilder(mapR2.getStartPosition())
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(60.0, 17.5))
                .strafeToLinearHeading(new Vector2d(35.95, 27.0), new Rotation2d(Math.toRadians(0.0), Math.toRadians(90.0)))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(35.95, 50.0))
                .waitSeconds(2.5)
                .strafeToConstantHeading(new Vector2d(60.0, 17.5))
                .build();

        bot2.runAction(!isRed ? redTraj2 : blueTraj2);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .addEntity(bot2)
                .start();


    }
}
