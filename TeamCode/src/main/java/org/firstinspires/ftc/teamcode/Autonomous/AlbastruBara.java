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


@Autonomous(name="AlbastruBara", group = "Autonomous")

public class AlbastruBara extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);
        BratGheara brat = new BratGheara(hardwareMap);

        Pose2d pose = new Pose2d(-23.6, 70.3, -1.56);    //heading era 80.1
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        AccelConstraint accFast=new ProfileAccelConstraint(-100.0,100.0);
        AccelConstraint accSlow = new ProfileAccelConstraint(-80.0, 80.0);

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
                .strafeToConstantHeading(new Vector2d(-0, 37), speedFast, accFast);     // -40

        TrajectoryActionBuilder plecareDupaPreload=drumPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-46, 45), speedFast, accFast);

        TrajectoryActionBuilder ajungereSample1 = plecareDupaPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-52, 13), speedFast, accFast)         //trebuie testat si x = 52
                .strafeToConstantHeading(new Vector2d(-57, 13), speedFast, accFast);

        TrajectoryActionBuilder prepareSample1 = ajungereSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-57, 57), speedFast, accSlow);

//        TrajectoryActionBuilder ajungereSample2 = prepareSample1.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(57, -13), speedFast, accFast)
//                .strafeToConstantHeading(new Vector2d(67, -13), speedFast, accSlow);
//
//        TrajectoryActionBuilder prepareSample2 = ajungereSample2.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(67, -57), speedFast, accFast);

        TrajectoryActionBuilder prepareToTakeSample1 = prepareSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-59, 50), speedFast, accFast)
                .turnTo(80);

        TrajectoryActionBuilder takeSample1 = prepareToTakeSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-59, 57), speedFast, accFast);

        TrajectoryActionBuilder scoreSample1 = takeSample1.endTrajectory().fresh()
                .turnTo(1.65)
                .strafeToConstantHeading(new Vector2d(-10, 50), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(-4, 41), speedFast, accFast);     // - 42

        TrajectoryActionBuilder prepareToTakeSample2 = scoreSample1.endTrajectory().fresh()
                .turnTo(80)
                .strafeToConstantHeading(new Vector2d(-10, 50), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(-57, 63), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(-57, 66), speedFast, accFast);

        TrajectoryActionBuilder scoreSample2 = prepareToTakeSample2.endTrajectory().fresh()
                .turnTo(1.65)
                .strafeToConstantHeading(new Vector2d(-10, 58), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(-6, 49), speedFast, accFast); //-50

        TrajectoryActionBuilder parcheaza =scoreSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-10, 50), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(-57, 69), speedFast, accFast);

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
//        Action actToSample2 = ajungereSample2.build();
//        Action actPrepareSample2 = prepareSample2.build();
        Action actPrepareToTakeSample1 = prepareToTakeSample1.build();
        Action actTakeSample1 = takeSample1.build();
        Action actScoreSample1 = scoreSample1.build();
        Action actPrepareToTakeSample2 = prepareToTakeSample2.build();
        Action actScoreSample2 = scoreSample2.build();
        Action actParcare = parcheaza.build();

        //aici se executa toate actiunile

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(
                                actPreload,
                                glisiera.glisieraSusJumate()
                        ),

                        glisiera.glisieraOutake(),
                        // aici lasare gheara
                        actPlecareDupaPreload,

                        new ParallelAction(
                                gheara.lasareGheara(),
                                actToSample1,
                                glisiera.glisieraJos()
                        ),

                        actPrepareSample1,

                        //actToSample2,
                        //actPrepareSample2,

                        new ParallelAction(
                                actPrepareToTakeSample1,
                                glisiera.glisieraJos()
                        ),

                        new SleepAction(0.4),
                        actTakeSample1,
                        brat.prindeSampleDePePerete(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.4),
                        new ParallelAction(
                                gheara.ridicareGhearaBrat(),
                                actScoreSample1,
                                glisiera.glisieraSusJumate()
                        ),
                        glisiera.glisieraOutake(),

                        new ParallelAction(
                                actPrepareToTakeSample2,
                                glisiera.glisieraJos(),
                                gheara.lasareGheara()
                        ),

                        brat.prindeSampleDePePerete(),
                        new SleepAction(0.4),
                        gheara.prindereGheara(),
                        new SleepAction(0.4),

                        new ParallelAction(
                                gheara.ridicareGhearaBrat(),
                                actScoreSample2,
                                glisiera.glisieraSusJumate()
                        ),
                        glisiera.glisieraOutake(),
                        gheara.lasareGheara(),
                        new SleepAction(0.3),
                        actParcare

                )
        );

    }
}
