package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 4420.985173528618)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(17.8, 70.1, -89.5)) //pt albasru -20 66 80
                .strafeTo(new Vector2d(60.3,61.2)) // pt albastru -10 27.7, math....0
                    //    .strafeToConstantHeading(new Vector2d(-68.9,69.1))//pt albastru -68.9 69.1
                        .turnTo(358.8)//180
                        .lineToX(66.7)
                        .lineToX(61.5)
                                .turnTo(-89.5)
                                .strafeToConstantHeading(new Vector2d(54,58))
                                .turnTo(88.4)
                                .strafeToConstantHeading(new Vector2d(61.3,62))
                               /* .turnTo(-89.5)
                                .strafeToConstantHeading(new Vector2d(44.1,56.2))
                                .strafeToConstantHeading(new Vector2d(44.1,-10))
                                //.turnTo(179.9)
                .turnTo(179.1)
                                .lineToX(16)*/



                                //.strafeTo(new Vector2d(-23.8,-69.4))
                               // .strafeTo(new Vector2d(55.6,-62.9))
                       // .strafeTo(new Vector2d(67.5,-67.3))
                        // .splineTo(new Vector2d(-16.0,-10.8),Math.toRadians(0))
                       // .strafeTo(new Vector2d(67.5,-67.3))
                /* pt rosu continuare
                .turnTo(89.5)
                                .strafeTo(new Vector2d(-47.2,-37.4))
                                .lineToX(-54.9)
                                .turnTo(179.8)//90.69
                                .lineToX(-62.4)
                 */

                .build());

    Image img=null;
    try{img= ImageIO.read(new File("C:/Users/The Emperor RO124/Documents/field2.jpeg"));}
    catch(IOException e){}

    //    meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}