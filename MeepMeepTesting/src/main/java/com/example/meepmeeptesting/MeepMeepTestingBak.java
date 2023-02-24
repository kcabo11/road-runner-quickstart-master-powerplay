package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBak {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40.25, -62.375, Math.toRadians(90)))
                                //.forward(22)

                                .lineToSplineHeading(new Pose2d(29, -28, Math.toRadians(120)))
                                .lineToSplineHeading(new Pose2d(35,-36, Math.toRadians(90)))
//                                .back(10)
//                                .turn(Math.toRadians(-30))
                                .forward(26)
                                .lineToSplineHeading(new Pose2d(60.5, -13.75, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(30,-16.5, Math.toRadians(215)))
                                .lineToSplineHeading(new Pose2d(60.5, -13.75, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(30,-16.5, Math.toRadians(215)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}