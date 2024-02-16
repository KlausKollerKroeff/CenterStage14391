package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.sun.org.apache.bcel.internal.generic.SWITCH;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

enum Route {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT,
    UNRECOGNIZED_ROUTE

}

public class MeepMeepTesting {
    public static final Route ROUTE = Route.RED_LEFT;
    public static final double DELAY = 0.5;
    public static final double MAXVEL = 60;
    public static final double MAXACCEL = 60;
    public static final double MAXANGVEL = Math.toRadians(180);
    public static final double MAXANGACCEL = Math.toRadians(180);
    public static final double TRACKWIDTH = 15;

    public static void main(String[] args) throws Exception {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;

        switch (ROUTE) {
            case RED_LEFT:
                myBot = redLeft (meepMeep);
                break;
            case RED_RIGHT:
                myBot = redRight(meepMeep);
                break;
            case BLUE_LEFT:
                myBot = blueLeft(meepMeep);
                break;
            case BLUE_RIGHT:
                myBot = blueRight(meepMeep);
                break;
            case UNRECOGNIZED_ROUTE:
                myBot = unrecognizedRoute(meepMeep);
        }

        Image img = null;
        try {
            img = ImageIO.read(new File("field-2024-official.png"));
        } catch (IOException e) {
        }

        meepMeep.setBackground(img)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();



    }
    private static RoadRunnerBotEntity redLeft (MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaulBotBuilder (meepMeep)
                //setando as drive constants
                .setConstraints(MAXVEL,MAXACCEL,MAXANGVEL,MAXANGACCEL,TRACKWIDTH)

                MecanumDrive(new Pose2d(11.5,60, Math.toRadians(180)))
                        .lineToY(0)
                        .waitSeconds(DELAY)
                        .setTangent(Math.toRadians(90))
                        .waitSeconds(DELAY)
                        .lineToX(-30)
                        .waitSeconds(DELAY)
                        .setTangent(Math.toRadians(90))
                        .waitSeconds(DELAY)
                        .lineToY(50)
                        .waitSeconds(DELAY)
                        .setTangent(Math.toRadians(90))
                        .waitSeconds(DELAY)
                        .build();
                );
        return myBot;
    }
    private static RoadRunnerBotEntity redLeft (MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaulBotBuilder (meepMeep)
                //setando as drive constants
                .setConstraints(MAXVEL,MAXACCEL,MAXANGVEL,MAXANGACCEL,TRACKWIDTH)

            MecanumDrive(new Pose2d(61,-36, Math.toRadians(180)))
}
