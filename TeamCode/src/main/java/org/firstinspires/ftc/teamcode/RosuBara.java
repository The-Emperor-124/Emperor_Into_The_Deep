package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
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
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import SubSystems.Gheara;
import SubSystems.Glisiera;


@Autonomous(name="RosuBara", group = "Autonomous")
public class RosuBara extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        Pose2d pose = new Pose2d(23.6, -70.3, 1.56);    //heading era 80.1
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

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

        //prima actiune este cea in care mergem la bara sa punem preload
        TrajectoryActionBuilder drumPreload = drive.actionBuilder(pose)
                .strafeToConstantHeading(new Vector2d(0, -40), speedFast, accFast);

        TrajectoryActionBuilder plecareDupaPreload=drumPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(46, -45), speedFast, accFast);

        TrajectoryActionBuilder ajungereSample1 = plecareDupaPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(52, -13), speedFast, accFast)         //trebuie testat si x = 52
                .strafeToConstantHeading(new Vector2d(57, -13), speedFast, accSlow);

        TrajectoryActionBuilder prepareSample1 = ajungereSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(57, -57), speedFast, accFast);

        TrajectoryActionBuilder ajungereSample2 = prepareSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(57, -13), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(67, -13), speedFast, accSlow);

        TrajectoryActionBuilder prepareSample2 = ajungereSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(67, -57), speedFast, accFast);

        TrajectoryActionBuilder prepareToTakeSample1 = prepareSample2.endTrajectory().fresh()
                        .strafeToConstantHeading(new Vector2d(59, -51), speedFast, accFast)
                        .turnTo(80);
        
        TrajectoryActionBuilder scoreSample1 = prepareToTakeSample1.endTrajectory().fresh()
                        .turnTo(1.65)
                        .strafeToConstantHeading(new Vector2d(0, -29), speedFast, accFast);

        Actions.runBlocking(gheara.ridicareGhearaBrat());
        Actions.runBlocking(gheara.prindereGheara());


        // Wait for the start signal
        waitForStart();


        if(isStopRequested()){
            return;
        }

        // aici compilam actiunile ca sa nu mai piarda timp

        Action actPreload = drumPreload.build();
        Action actPlecareDupaPreload = plecareDupaPreload.build();
        Action actToSample1 = ajungereSample1.build();
        Action actPrepareSample1 = prepareSample1.build();
        Action actToSample2 = ajungereSample2.build();
        Action actPrepareSample2 = prepareSample2.build();
        Action actPrepareToTakeSample1 = prepareToTakeSample1.build();
        Action actScoreSample1 = scoreSample1.build();


        //aici se executa toate actiunile

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(
                                actPreload,
                                glisiera.glisieraSusJumate()
                        ),

                        glisiera.glisieraOutake(),
                        gheara.lasareGheara(),
                        actPlecareDupaPreload,

                        new ParallelAction(
                                actToSample1,
                                glisiera.glisieraJos()
                        ),

                        actPrepareSample1,
                        actToSample2,
                        actPrepareSample2,
                        actPrepareToTakeSample1,
                        
                        new SleepAction(0.2),
                        gheara.prindeSampleBara(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                gheara.ridicareGhearaBrat(),
                                actScoreSample1,
                                glisiera.glisieraSusJumate()
                        ),
                        glisiera.glisieraOutake(),
                        gheara.lasareGheara()

                )
        );

    }
}
