package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // trajectoryAction1 from BlueSideTestAuto.
        /*
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.8, 61.7, Math.toRadians(90)))
                .lineToY(35)
 //               .setTangent(Math.toRadians(0))
 //               .lineToX(16)
                .waitSeconds(2)
                        .lineToY(50)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(0))
                .waitSeconds(3)
                .build());
         */

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14.76, -62.89, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(6.44, -35.0, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(2)
                .lineToY(-48)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(37.11, -37.11, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}