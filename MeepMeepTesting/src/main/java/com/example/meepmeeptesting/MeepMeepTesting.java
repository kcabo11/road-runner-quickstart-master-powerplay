package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34.25, -62.375, Math.toRadians(90)))
                                //.forward(22)
                                .splineTo(new Vector2d(-30,-29), Math.toRadians(60))
                                .waitSeconds(1.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(-36, -11), Math.toRadians(0))

                                .forward(26)
                                .waitSeconds(0.5)
                                .back(25)

                                .lineToSplineHeading(new Pose2d(-28,-14, Math.toRadians(310)))
                                .waitSeconds(1.5)

                                .lineToSplineHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                                .forward(26)
                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(-46,-6, Math.toRadians(270)))

                                .forward(8)
                                .waitSeconds(1.5)
                                .back(3)

                                .lineToSplineHeading(new Pose2d(-36, -11, Math.toRadians(180)))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(36, -9), Math.toRadians(180))
//                                .forward(25)
//                                .back(25)
//
//                                .lineToSplineHeading(new Pose2d(30,-16, Math.toRadians(220)))
//                                .splineToLinearHeading(new Pose2d(36,-9, Math.toRadians(0)), Math.toRadians(0))
//                                .forward(25)
//                                .back(25)
//                                .lineToSplineHeading(new Pose2d(30,-16, Math.toRadians(220)))
//                                .splineToLinearHeading(new Pose2d(36,-9, Math.toRadians(0)), Math.toRadians(0))
//
//                                .splineToLinearHeading(new Pose2d(36,-9), Math.toRadians(130))
//                                .back(10)
//                                .lineToLinearHeading(new Pose2d(48,-24, Math.toRadians(270)))
////                                .back(25)

                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}