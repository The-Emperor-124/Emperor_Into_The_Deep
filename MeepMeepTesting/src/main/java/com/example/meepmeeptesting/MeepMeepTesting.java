package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-39.9, -70.1, 89.5)) //17.8 70.1
               /* .strafeToConstantHeading(new Vector2d(-39.9, -64))
                .strafeToConstantHeading(new Vector2d(-61.5, -63))  //63.5
//.strafeToConstantHeading(new Vector2d(-64, -64))
                .turnTo(79.3)
                .strafeToConstantHeading(new Vector2d(-57, -57))
                .turnTo(1.56)
                .strafeToConstantHeading(new Vector2d(-49.5, -52))
                .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-58.5, -60))
                .strafeToConstantHeading(new Vector2d(-59, -59))
                .turnTo(1.56)
                .strafeToConstantHeading(new Vector2d(-61, -52))
                .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-64, -64))*/
                .strafeToConstantHeading(new Vector2d(-59,-59))
                .turnTo(84.6)
                .strafeToConstantHeading(new Vector2d(-55.8,-32))
                .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-65,-65))
                .strafeToConstantHeading(new Vector2d(-45, -15))
                .turnTo(0 * Math.PI)
                        //.setTangent(-90)
                        //.splineToConstantHeading(new Vector2d(45,-12.5),90)
                /*.setTangent(-3 * Math.PI/2)        // 3.3 era inainte si mergea cu actiune singulara
                .splineToSplineHeading(new Pose2d(5,-40,-3 * Math.PI/2),-269)
                        //.turnTo(1.56)
                .setTangent(-Math.PI / 2)        // cu -80 nu se rotea suficient
                .splineToLinearHeading(new Pose2d(57, -52, -Math.PI / 2),269)
              //  .turnTo(-1.56)
                .setTangent(-3 * Math.PI/2)
                .splineToSplineHeading(new Pose2d(10,-40,-3 * Math.PI/2),-269)//3 * Math.PI/2

                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(57, -58, -Math.PI/2),269)
               // .turnTo(-1.56)

                .setTangent(-3 * Math.PI/2)
                .splineToSplineHeading(new Pose2d(10,-40,-3 * Math.PI/2),-269)
               // .setTangent(80)
                //.splineToLinearHeading(new Pose2d(57, -58, 80),269)
              /*  .setTangent( -3 * Math.PI / 2)        // cu -80 nu se rotea suficient
                .splineToLinearHeading(new Pose2d(0, -37,- 3 * Math.PI/2),-269)

                /*

                 .setTangent( -3 * Math.PI / 2)        // cu -80 nu se rotea suficient
                .splineToLinearHeading(new Pose2d(0, -37,- 3 * Math.PI/2),-270)
                 */
//                .strafeToConstantHeading(new Vector2d(10,-52))
               /* .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(57, -58, -Math.PI/2),269)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(57, -56, -Math.PI/2),269)
                .setTangent(-3 * Math.PI/2)
                .splineToSplineHeading(new Pose2d(8,-40,-3 * Math.PI/2),-269)*/
//                .setTangent(-3.25 * Math.PI/2)
//                .splineToSplineHeading(new Pose2d(10,-35,-3.25 * Math.PI/2),90)
//                .strafeToConstantHeading(new Vector2d(8, -52))
//                .setTangent(80)
//                .splineToLinearHeading(new Pose2d(57, -58, 79.8),-90)
//                .setTangent(-3.25 * Math.PI/2)
//                .splineToSplineHeading(new Pose2d(10,-35,-3.25 * Math.PI/2),90)
//                .strafeToConstantHeading(new Vector2d(7, -52))
//                .setTangent(80)
//                .splineToLinearHeading(new Pose2d(57, -58, 80),-270)


                //.setTangent(3.25 * Math.PI/2)
               // .splineToLinearHeading(new Pose2d(-57, 58, -79.8),-270)
                       // .splineToSplineHeading(new Pose2d(-57,57,-1.56),90)
                      /*  .setTangent(3.2 * Math.PI / 2)
                        .splineToLinearHeading(new Pose2d(-6, 65, 3.2 * Math.PI/2),-270)
                        .setTangent(-80)
                         .splineToLinearHeading(new Pose2d(-57, 56, -80),-270)
                        //.splineToSplineHeading(new Pose2d(-0,45,-1.56), -270)
                .setTangent(3.2 * Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-6, 65, 3.2 * Math.PI/2),-270)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-57, 56, Math.PI/2),-270)*/



                .build());

    Image img=null;
    try{img= ImageIO.read(new File("C:/Users/The Emperor RO124/Documents/field2.jpeg"));}
    //try{img= ImageIO.read(new File("C:/Users/The Emperor RO124Downloads/field_2024.jpeg"));}
    catch(IOException e){}

    //    meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

// aici e ce era in mybot


/*

//                                .strafeToConstantHeading(new Vector2d(63,45))
//                                .turnTo(88.4)
//                                .strafeToConstantHeading(new Vector2d(67.5,66))
//                                .lineToX(61.5)
//                                .strafeToConstantHeading(new Vector2d(19.9,8.6))
                //        .strafeToConstantHeading(new Vector2d(17,68))//pt albasru -20 66 80
              //  .strafeTo(new Vector2d(65.7,62)) // pt albastru -10 27.7, math....0
                    //    .strafeToConstantHeading(new Vector2d(-68.9,69.1))//pt albastru -68.9 69.1
                       /* .turnTo(358.8)//180
                        .lineToX(66.7)
                        .lineToX(61.5)
                                .turnTo(-89.5)
                                .strafeToConstantHeading(new Vector2d(54,53))
                                .turnTo(88.4)
                                .strafeToConstantHeading(new Vector2d(62.2,60.5))
                .lineToX(67.9)
                .lineToX(61.5)
                                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(66,52.7))
                .strafeToConstantHeading(new Vector2d(60.3, 61.2))
                .turnTo(88.4)
                .lineToX(67)
                .lineToX(61.5)
                                .strafeToConstantHeading(new Vector2d(65,53))



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


/*
// comenzi pentru rosu bara
// pozitie start : new Pose2d(23.6, -70.3, 1.56)

                      .strafeToConstantHeading(new Vector2d(0, -25))
                .strafeToConstantHeading(new Vector2d(20, -60))
                .strafeToConstantHeading(new Vector2d(51, -13))
                .strafeToConstantHeading(new Vector2d(57, -13))
                .strafeToConstantHeading(new Vector2d(57, -57))
                .strafeToConstantHeading(new Vector2d(57, -13))
                .strafeToConstantHeading(new Vector2d(67, -13))
                .strafeToConstantHeading(new Vector2d(67, -57))
                .strafeToConstantHeading(new Vector2d(57, -57))
                .turnTo(80.1)
                .turnTo(1.65)

                .strafeToConstantHeading(new Vector2d(10, -50))
                .strafeToConstantHeading(new Vector2d(0, -29))
                .strafeToConstantHeading(new Vector2d(10, -50))
                .strafeToConstantHeading(new Vector2d(58, -59))

 */


/*
    // comenzi rosu cos
    // pozitie start new Pose2d(-17.8, -70.1, 1.56)


 */
/*    .strafeToConstantHeading(new Vector2d(39.9,64))
                        .strafeToConstantHeading(new Vector2d(63.5,61.5))
                        .strafeToConstantHeading(new Vector2d(64,65))
                .turnTo(358.9)
                .strafeToConstantHeading(new Vector2d(59, 59))
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(53, 53))
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(65, 65))
                .strafeToConstantHeading(new Vector2d(59, 59))
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(65, 51 ))
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(65, 65))
                //
                .strafeToConstantHeading(new Vector2d(59,59))
                .turnTo(270.2)
                .strafeToConstantHeading(new Vector2d(53,50.8))
               // .turnTo(Math.toRadians(180))
                .turnTo(95)
                .strafeToConstantHeading(new Vector2d(65,65))
                .strafeToConstantHeading(new Vector2d(50, 15))
                .turnTo(60)
                //.turnTo(-0.1)
                .strafeToConstantHeading(new Vector2d(30, 5))

              //  .turnTo(95)
                      //  .strafeToConstantHeading(new Vector2d(65,65))
                //.turnTo(Math.toRadians(-1.56))
              //  .turnTo(-79.1)
                //.strafeToConstantHeading(new Vector2d(39.1, 64))//17
                  //              .strafeToConstantHeading(new Vector2d(64.5,61.5))
//                                .strafeToConstantHeading(new Vector2d(-0,37))
//                                .strafeToConstantHeading(new Vector2d(-46,45))
//                                .strafeToConstantHeading(new Vector2d(-52,13))
//                                .strafeToConstantHeading(new Vector2d(-57,13))
//                                .strafeToConstantHeading(new Vector2d(-57,57))
//                                .strafeToConstantHeading(new Vector2d(-59,50))
//                                .turnTo(-80)
//                                .strafeToConstantHeading(new Vector2d(-59,57))
//                                .turnTo(-1.65)
//                                .strafeToConstantHeading(new Vector2d(-10,50))
//                                .strafeToConstantHeading(new Vector2d(-4,41))
//                                .turnTo(80)
//                                .strafeToConstantHeading(new Vector2d(-10,50))
//                                .strafeToConstantHeading(new Vector2d(-57,63))
//                                .strafeToConstantHeading(new Vector2d(-57,66))
//                                .turnTo(-1.65)
//                                .strafeToConstantHeading(new Vector2d(-10,58))
//                                .strafeToConstantHeading(new Vector2d(-8,49))
//                                .strafeToConstantHeading(new Vector2d(-10,50))
//                                .strafeToConstantHeading(new Vector2d(-57,69))
             /*   .strafeToConstantHeading(new Vector2d(-17, -64))
                .strafeToConstantHeading(new Vector2d(-65, -61.5))
                        .turnTo(79.3)
                .strafeToConstantHeading(new Vector2d(-59, -63))
                        .turnTo(1.56)
                .strafeToConstantHeading(new Vector2d(-55, -53))
                        .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-65, -65))
                .strafeToConstantHeading(new Vector2d(-59, -63))
                        .turnTo(1.56)
                .strafeToConstantHeading(new Vector2d(-67, -52))
                        .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-65, -65))
                                .strafeToConstantHeading(new Vector2d(-69,-65))
                //.strafeToConstantHeading(new Vector2d(57, -57))
                //.turnTo(80.1)
                //.turnTo(1.65)

              /*  .strafeToConstantHeading(new Vector2d(10, -50))
                .strafeToConstantHeading(new Vector2d(0, -29))
                .strafeToConstantHeading(new Vector2d(10, -50))
                .strafeToConstantHeading(new Vector2d(58, -59))*/