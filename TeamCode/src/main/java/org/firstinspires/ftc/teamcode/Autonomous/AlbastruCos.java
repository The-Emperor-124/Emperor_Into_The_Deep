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

import org.firstinspires.ftc.teamcode.Subsystems.BratGheara;
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
        BratGheara brat = new BratGheara(hardwareMap);
        Glisiera glisisera = new Glisiera(hardwareMap);

        Pose2d pozitieInitiala = new Pose2d(17.8, 70.1, -89.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pozitieInitiala);

        AccelConstraint accFast=new ProfileAccelConstraint(-100, 140);
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
                .strafeToConstantHeading(new Vector2d(18,64), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(65, 62), speedFast, accFast)  // era 66 cand lovea 60.3, 63.2
                .turnTo(358.9)
                .strafeToConstantHeading(new Vector2d(66, 66), speedFast, accSlow);

        TrajectoryActionBuilder prevDrumToSample1 = scorePreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(59,63), speedFast, accFast);

        TrajectoryActionBuilder drumToSample1 = prevDrumToSample1.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(56, 54.5), speedFast, accFast);     //era 53

        TrajectoryActionBuilder scoreSample1 = drumToSample1.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(66, 66), speedFast, accFast);         // era 68 68

        TrajectoryActionBuilder prevDrumToSample2 = scoreSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(59, 63), speedFast, accFast);

        TrajectoryActionBuilder drumToSample2  = prevDrumToSample2.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(67, 54.5), speedFast, accFast);     // era 53

        TrajectoryActionBuilder scoreSample2 = drumToSample2.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(67.5, 66), speedFast, accFast);

        TrajectoryActionBuilder prevParcare = scoreSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(50, 15), speedFast, accSlow)
                .turnTo(3);

        TrajectoryActionBuilder parcare = prevParcare.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(30, 5), speedSlow, accSlow);

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

        Action actPrevParcare = prevParcare.build();
        Action actParcare = parcare.build();

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
                        brat.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                brat.ridicareGhearaBrat(),
                                glisisera.glisieraSusCosRapid(),
                                actScoreSample1
                        ),
                        gheara.lasareGheara(),

                        actPrevRoadToSample2,
                        new ParallelAction(
                                actRoadToSample2,
                                glisisera.glisieraJos()
                        ),
                        brat.pozitioneazaBratPentruSampleDePeTeren(),
                        new SleepAction(0.5),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                brat.ridicareGhearaBrat(),
                                actScoreSample2,
                                glisisera.glisieraSusCosRapid()
                        ),
                        gheara.deschideGhearaPentruSamplePerete(),
                        actPrevParcare,
                        new ParallelAction(
                                actParcare,
                                glisisera.glisieraParcareBara()
                        ),
                        brat.pozitioneazaBratParcareCos(),
                        new SleepAction(0.2)

                )
        );


    }
}