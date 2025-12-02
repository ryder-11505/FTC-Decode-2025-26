package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart

interface Map {

    val parkPosition: Pose2d
    val startPosition: Pose2d
    val intakePos3: Pose2d
    val intakePos2: Pose2d
    val intakePos1: Pose2d
    val shootPos2: Pose2d
    val shootPos : Pose2d

//    GAME SPECIFIC OPTIMISATION


    val intakeParts: Array<AutoPart>
    val parkParts: Array<AutoPart>
    val shootParts: Array<AutoPart>
}