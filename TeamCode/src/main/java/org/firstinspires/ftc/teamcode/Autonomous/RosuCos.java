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


@Autonomous(name="RosuCos", group = "Autonomous")

public class RosuCos extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Gheara gheara = new Gheara(hardwareMap);
        BratGheara brat = new BratGheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        Pose2d pozitieInitiala= new Pose2d(-17.8, -70.1, 1.56);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pozitieInitiala);

        AccelConstraint accFast=new ProfileAccelConstraint(-80.0,80.0);
        AccelConstraint accSlow = new ProfileAccelConstraint(-30.0, 30.0);

        VelConstraint speedFast= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(80.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        VelConstraint speedSlow= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(pozitieInitiala)
                .strafeToConstantHeading(new Vector2d(-17, -64), speedSlow, accSlow)
                .strafeToConstantHeading(new Vector2d(-65, -63), speedSlow, accSlow)
                .turnTo(79.3);

        TrajectoryActionBuilder prevDrumToSample1 = scorePreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-59, -63), speedFast, accFast);

        TrajectoryActionBuilder drumToSample1 = prevDrumToSample1.endTrajectory().fresh()
                .turnTo(1.56)
                .strafeToConstantHeading(new Vector2d(-55, -53), speedFast, accFast);

        TrajectoryActionBuilder scoreSample1 = drumToSample1.endTrajectory().fresh()
                .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-65, -65), speedFast, accFast);  //-66 cu -66 dadea doar putin //-68, -67

        TrajectoryActionBuilder prevDrumToSample2 = scoreSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-59, -63), speedFast, accFast);

        TrajectoryActionBuilder drumToSample2 = prevDrumToSample2.endTrajectory().fresh()
                .turnTo(1.56)
                .strafeToConstantHeading(new Vector2d(-67, -51), speedFast, accFast);

        TrajectoryActionBuilder scoreSample2 = drumToSample2.endTrajectory().fresh()
                .turnTo(79.1)
                .strafeToConstantHeading(new Vector2d(-65, -65), speedFast, accFast);

        TrajectoryActionBuilder prevParcare = scoreSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-50, -15), speedFast, accFast)
                .turnTo(7)
                .turnTo(-0.1);

        TrajectoryActionBuilder parcare = prevParcare.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-30, -5),speedSlow ,accSlow);

        Actions.runBlocking(brat.ridicareGhearaBrat());
        Actions.runBlocking(gheara.prindereGheara());
        //Actions.runBlocking(brat.pozitioneazaBratParcareCos());

        waitForStart();


        if(isStopRequested()){
            return;
        }

        //compilam actiunile inainte de rulare
        Action actScorePreload = scorePreload.build();

        Action actPrevDrumToSample1 = prevDrumToSample1.build();
        Action actDrumToSample1 = drumToSample1.build();
        Action actScoreSample1 = scoreSample1.build();

        Action actPrevDrumToSample2 = prevDrumToSample2.build();
        Action actDrumToSample2 = drumToSample2.build();
        Action actScoreSample2 = scoreSample2.build();

        Action actPrevParcare = prevParcare.build();
        Action actParcare = parcare.build();


        //aici se executa actiunile
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actScorePreload,
                                glisiera.glisieraSusCosRapid()
                        ),
                        gheara.lasareGheara(),

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