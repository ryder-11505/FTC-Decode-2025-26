package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType

class InsideLeaveRed : Map {
    override val startPosition = Pose2d(33.9, 12.2, 270.0 * Math.PI / 180)

    override val shootPos = Pose2d(-7.3, 4.7, 35.0 * Math.PI / 180)
    override val shootPos2 = Pose2d(1.0, -28.0, 0.0 * Math.PI / 180)

    override val intakePos1 = Pose2d(-6.3, 16.0, 0.0 * Math.PI / 180)
    override val intakePos2 = Pose2d(6.7, 16.0, 0.0 * Math.PI / 180)
    override val intakePos3 = Pose2d(19.9, 16.0, 0.0 * Math.PI / 180)

    override val parkPosition = Pose2d(30.0, -30.0, 0.0 * Math.PI / 180)


    override val intakeParts = arrayOf(
        AutoPart(PartType.STRAFE, intakePos1),
        AutoPart(PartType.STRAFE, Pose2d(-12.0, 53.0, 0.0)),
    )

    override val shootParts = arrayOf(
        AutoPart(PartType.STRAFE, shootPos),
    )
//  Then got to specimen collect position, collect specimen, then repeat this previous action


    override val parkParts = arrayOf(
        AutoPart(PartType.STRAFE_TO, parkPosition),
    )
}