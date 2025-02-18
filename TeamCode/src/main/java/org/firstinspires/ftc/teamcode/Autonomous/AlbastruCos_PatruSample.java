package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.Subsystems.BratGheara;
import org.firstinspires.ftc.teamcode.others.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Gheara;
import org.firstinspires.ftc.teamcode.Subsystems.Glisiera;


@Autonomous(name="AlbastruCos_PatruSample", group = "Autonomous")

public class AlbastruCos_PatruSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Gheara gheara = new Gheara(hardwareMap);
        BratGheara brat = new BratGheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);


        Pose2d pozitieInitiala= new Pose2d(39.9, 70.1, -89.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pozitieInitiala);

        AccelConstraint accFast=new ProfileAccelConstraint(-105.0,105.0);
        AccelConstraint accSlow = new ProfileAccelConstraint(-45.0, 90.0);

        VelConstraint speedFast= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(130.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        VelConstraint speedSlow= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(80.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        // preload

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(pozitieInitiala)
                .strafeToConstantHeading(new Vector2d(39.9, 64), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(61.5, 61.5), speedFast, accFast)  //63.5
                .strafeToConstantHeading(new Vector2d(64, 64), speedFast, accFast)
                .turnTo(358.9);


        // sample 1

        //aici se da in spate pana sa coboare glisiera
        TrajectoryActionBuilder prevDrumToSample1 = scorePreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(59, 59), speedFast, accFast);

        TrajectoryActionBuilder drumToSample1 = prevDrumToSample1.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(52, 52), speedFast, accFast);

        TrajectoryActionBuilder scoreSample1 = drumToSample1.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(65, 65), speedFast, accFast);  //-66 cu -66 dadea doar putin //-68, -67


        // sample 2

        //aici se da in spate pana sa coboare glisiera
        TrajectoryActionBuilder prevDrumToSample2 = scoreSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(59, 59), speedFast, accFast);

        TrajectoryActionBuilder drumToSample2 = prevDrumToSample2.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(64, 52 ), speedFast, accFast);        // aici am modificat y era 52 -> 18.02

        TrajectoryActionBuilder scoreSample2 = drumToSample2.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(64, 64), speedFast, accFast); // x a fost 65


        // sample 3


        //aici se da in spate pana sa coboare glisiera
        TrajectoryActionBuilder prevDrumToSample3= scoreSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(59,59),speedFast,accFast);

        TrajectoryActionBuilder drumToSample3=prevDrumToSample3.endTrajectory().fresh()
                .turnTo(270.1)
                .strafeToConstantHeading(new Vector2d(55.8,32),speedFast,accFast);// 55.5 mergea la un moment dat// 53.5//69.7

        TrajectoryActionBuilder scoreSample3=drumToSample3.endTrajectory().fresh()
               // .turnTo(Math.toRadians(180))
                .turnTo(95)
                .strafeToConstantHeading(new Vector2d(65,65),speedFast,accFast);

        // parcare

        TrajectoryActionBuilder prevParcare = scoreSample3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(50, 15), speedFast, accFast)
                .turnTo(60);
               // .turnTo(-0.1);

        TrajectoryActionBuilder parcare = prevParcare.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(30, 5),speedFast ,accFast);

        Actions.runBlocking(brat.ridicareGhearaBrat());
        Actions.runBlocking(gheara.prindereGheara());

        waitForStart();

        if(isStopRequested()){
            return;
        }

        //compilam actiunile inainte de rulare

        // preload

        Action actScorePreload = scorePreload.build();


        // sample 1

        Action actPrevDrumToSample1 = prevDrumToSample1.build();
        Action actDrumToSample1 = drumToSample1.build();
        Action actScoreSample1 = scoreSample1.build();


        // sample 2

        Action actPrevDrumToSample2 = prevDrumToSample2.build();
        Action actDrumToSample2 = drumToSample2.build();
        Action actScoreSample2 = scoreSample2.build();


        // sample 3

        Action actPrevDrumToSample3=prevDrumToSample3.build();
        Action actDrumToSample3=drumToSample3.build();
        Action actScoreSample3=scoreSample3.build();





        Action actPrevParcare = prevParcare.build();
        Action actParcare = parcare.build();


        //aici se executa actiunile
        Actions.runBlocking(
                new SequentialAction(

                        // score preload

                        new ParallelAction(
                                actScorePreload,
                                glisiera.glisieraSusCosRapid()
                        ),
                        // maybe calculat timpul si bagat cu sequential action in parallel + sleep
                        gheara.lasareGheara(),


                        // sample 1

                        actPrevDrumToSample1,
                        new ParallelAction(
                                actDrumToSample1,
                                glisiera.glisieraJos()
                        ),
                        brat.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                brat.ridicareGhearaBrat(),
                                glisiera.glisieraSusCosRapid(),
                                actScoreSample1
                        ),
                        gheara.lasareGheara(),


                        // sample 2

                        actPrevDrumToSample2,
                        new ParallelAction(
                                actDrumToSample2,
                                glisiera.glisieraJos()
                        ),
                        brat.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.5),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                brat.ridicareGhearaBrat(),
                                actScoreSample2,
                                glisiera.glisieraSusCosRapid()
                        ),
                        gheara.deschideGhearaPentruSamplePerete(),


                        // sample 3

                        actPrevDrumToSample3,
                        new ParallelAction(
                                actDrumToSample3,
                                glisiera.glisieraJos()
                        ),
                        brat.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.5),
                        gheara.inchideGhearaPentruColectareSamplePerete(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                brat.ridicareGhearaBrat(),
                                actScoreSample3,
                                glisiera.glisieraSusCosRapid()
                        ),
                        gheara.lasareGheara(),


                        // daca o mai fi timp de parcare

                        actPrevParcare,
                        new ParallelAction(
                                actParcare,
                                glisiera.glisieraParcareBara()
                        ),
                        brat.pozitioneazaBratParcareCos(),
                        new SleepAction(0.2)

                )
        );

    }
}