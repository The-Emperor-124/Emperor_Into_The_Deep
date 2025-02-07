package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Gheara;
import org.firstinspires.ftc.teamcode.Subsystems.Glisiera;
import org.firstinspires.ftc.teamcode.others.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "AlbastruCos", group = "Autonomous")
//@Disabled
public class  AlbastruCos extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiem Gheara si Glisiera -> aka subsistemele robotului
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisisera = new Glisiera(hardwareMap);

        Pose2d pozitieInitiala = new Pose2d(17.8, 70.1, -89.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pozitieInitiala);

        AccelConstraint accFast=new ProfileAccelConstraint(-100, -140);
        AccelConstraint accSlow = new ProfileAccelConstraint(-45.0, 90.0);

        VelConstraint speedFast= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        VelConstraint speedSlow= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(pozitieInitiala)
                .strafeToConstantHeading(new Vector2d(15,68))
                .strafeToConstantHeading(new Vector2d(67.5, 63.5), speedFast, accSlow)  // era 66 cand lovea 60.3, 63.2
                .turn(358.8);

        TrajectoryActionBuilder prevDrumToSample1 = scorePreload.endTrajectory().fresh()
                .lineToX(61.5);

        TrajectoryActionBuilder drumToSample1 = prevDrumToSample1.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(55, 53), speedFast, accFast);

        TrajectoryActionBuilder scoreSample1 = drumToSample1.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(68, 68), speedFast, accFast);

        TrajectoryActionBuilder prevDrumToSample2 = scoreSample1.endTrajectory().fresh()
                .lineToX(61.5);

        TrajectoryActionBuilder drumToSample2  = prevDrumToSample2.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(66, 53), speedFast, accFast);

        TrajectoryActionBuilder scoreSample2 = drumToSample2.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(67.5, 66), speedFast, accFast);


        // blocuri care se executa la init
        Actions.runBlocking(gheara.prindereGheara());
        Actions.runBlocking(gheara.ridicareGhearaBrat());

        waitForStart();

        if(isStopRequested()){
            return;
        }

        //compilam traiectoriile inainte de run
        Action actScorePreload = scorePreload.build();


        Action actPrevRoadToSample1 = prevDrumToSample1.build();
        Action actRoadToSample1 = drumToSample1.build();
        Action actScoreSample1 = scoreSample1.build();

        Action actPrevRoadToSample2 = prevDrumToSample2.build();
        Action actRoadToSample2 = drumToSample2.build();

        Action actScoreSample2 = scoreSample2.build();

        // aici se vor executa toate traiectoriile
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actScorePreload,
                                glisisera.glisieraSusCosRapid()
                        ),
                        gheara.lasareGheara(),

                        actPrevRoadToSample1,

                        new ParallelAction(
                                actRoadToSample1,
                                glisisera.glisieraJos()
                        ),
                        gheara.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                gheara.ridicareGhearaBrat(),
                                glisisera.glisieraSusCosRapid(),
                                actScoreSample1
                        ),
                        gheara.lasareGheara(),

                        actPrevRoadToSample2,

                        new ParallelAction(
                                actRoadToSample2,
                                glisisera.glisieraJos()
                        ),
                        gheara.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                gheara.ridicareGhearaBrat(),
                                actScoreSample2,
                                glisisera.glisieraSusCos()
                        ),
                        gheara.deschideGhearaPentruSamplePerete()
                )
        );


    }
}