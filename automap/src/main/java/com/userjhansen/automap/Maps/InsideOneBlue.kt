package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType


class InsideOneBlue : Map {
    override val startPosition = Pose2d(61.5, -17.5, Math.toRadians(180.0))

    override val shootPos = Pose2d(-7.3, -4.7, 135.2 * Math.PI / 180)
    override val shootPos2 = Pose2d(1.0, -28.0, 0.0 * Math.PI / 180)

    override val intakePos1 = Pose2d(-2.6, -13.0, 180.0 * Math.PI / 180)
    override val intakePos2 = Pose2d(10.2, -13.0, 180.0 * Math.PI / 180)
    override val intakePos3 = Pose2d(23.2, -13.0, 180.0 * Math.PI / 180)

    override val parkPosition = Pose2d(30.0, 30.0, 0.0 * Math.PI / 180)


    override val intakeParts = arrayOf(
        AutoPart(PartType.STRAFE, intakePos1),
        AutoPart(PartType.STRAFE, Pose2d(-2.6, -23.0, 180.0 * Math.PI / 18)),
    )

    override val shootParts = arrayOf(
        AutoPart(PartType.STRAFE, shootPos),
    )
//  Then got to specimen collect position, collect specimen, then repeat this previous action


    override val parkParts = arrayOf(
        AutoPart(PartType.STRAFE_TO, parkPosition),
    )
}