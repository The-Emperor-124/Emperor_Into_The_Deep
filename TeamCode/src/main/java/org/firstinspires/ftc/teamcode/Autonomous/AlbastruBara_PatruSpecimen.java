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

@Autonomous(name="AlbastruBara_PatruSpecimen", group = "Autonomous")

public class AlbastruBara_PatruSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);
        BratGheara brat = new BratGheara(hardwareMap);

        Pose2d pose = new Pose2d(-23.6, 70.3, -1.56);    //heading era 80.1
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        AccelConstraint accFast=new ProfileAccelConstraint(-130.0,130.0);
        AccelConstraint accSlow = new ProfileAccelConstraint(-80.0, 80.0);
        AccelConstraint superFast = new ProfileAccelConstraint(-150, 150);

        VelConstraint speedFast= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(130.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        VelConstraint speedSlow= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)

        ));


        // preload 1

        //prima actiune este cea in care mergem la bara sa punem preload
        TrajectoryActionBuilder drumPreload = drive.actionBuilder(pose)
                .strafeToConstantHeading(new Vector2d(-0, 32.5), speedFast, accFast);    // are 37 // -40

        TrajectoryActionBuilder plecareDupaPreload=drumPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 45), speedFast, accFast);    // 46 la x si jos la fel


        // pregatire inca 2 sample uri

        TrajectoryActionBuilder ajungereSample1 = plecareDupaPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 12.5), speedFast, accFast)         //trebuie testat si x = 52
                .strafeToConstantHeading(new Vector2d(-53, 12.5), speedFast, accFast);   // la 52 ia cu coltu   // - 57

        TrajectoryActionBuilder prepareSample1 = ajungereSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-57, 57), speedFast, accFast);

        TrajectoryActionBuilder ajungereSample2 = prepareSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-57, 13), speedFast, accFast)
                .strafeToConstantHeading(new Vector2d(-66, 13), speedFast, accFast);    // era - 67

        TrajectoryActionBuilder prepareSample2 = ajungereSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-67, 57), speedFast, accFast);



        // for specimen 2

        TrajectoryActionBuilder prepareToTakeSample1 = prepareSample2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-59, 50), speedFast, accFast)
                .turnTo(-80);


        TrajectoryActionBuilder takeSample1 = prepareToTakeSample1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-59, 57.5), speedFast, accFast);//57

        TrajectoryActionBuilder scoreSample1 = takeSample1.endTrajectory().fresh()
                .setTangent(3.25 * Math.PI/2)        // 3.3 era inainte si mergea cu actiune singulara
                .splineToSplineHeading(new Pose2d(-10,64,3.25 * Math.PI/2),270, speedFast, superFast)   // 65
                .strafeToConstantHeading(new Vector2d(-10,52), speedFast, superFast);

        TrajectoryActionBuilder prepareToTakeSample2 = scoreSample1.endTrajectory().fresh()
                .setTangent(Math.PI / 2)        // cu -80 nu se rotea suficient
                .splineToLinearHeading(new Pose2d(-57, 58, Math.PI/2),-270, speedFast, superFast);

        TrajectoryActionBuilder scoreSample2 = prepareToTakeSample2.endTrajectory().fresh()
                .setTangent(3.25 * Math.PI/2)
                .splineToSplineHeading(new Pose2d(-10,62,3.25 * Math.PI/2),270, speedFast, superFast)
                .strafeToConstantHeading(new Vector2d(-8, 52), speedFast, superFast);


        // sample 3
        TrajectoryActionBuilder prepareToTakeSample3 = scoreSample2.endTrajectory().fresh()
                .setTangent(-80)
                .splineToLinearHeading(new Pose2d(-57, 58, -80),-270, speedFast, superFast);

        TrajectoryActionBuilder scoreSample3 = prepareToTakeSample3.endTrajectory().fresh()
                .setTangent(3.25 * Math.PI/2)
                .splineToSplineHeading(new Pose2d(-10,62,3.25 * Math.PI/2),270, speedFast, superFast);



        //parcare

        TrajectoryActionBuilder parcheaza = scoreSample3.endTrajectory().fresh()
                .setTangent(-80)
                .splineToLinearHeading(new Pose2d(-57, 58, -80),-270, speedFast, superFast);






        // actiuni executate inainte de start program

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


        // actiuni de pregatire inca doua sample uri

        Action actToSample1 = ajungereSample1.build();
        Action actPrepareSample1 = prepareSample1.build();
        Action actToSample2 = ajungereSample2.build();
        Action actPrepareSample2 = prepareSample2.build();

        // score specimen 1
        Action actPrepareToTakeSample1 = prepareToTakeSample1.build();
        Action actTakeSample1 = takeSample1.build();
        Action actScoreSample1 = scoreSample1.build();

        // score specimen 2

        Action  actPrepareToTakeSample2 = prepareToTakeSample2.build();
        Action actScoreSample2 = scoreSample2.build();


        // score specimen 3

        Action actPrepareToTakeSample3 = prepareToTakeSample3.build();
        Action actScoreSample3 = scoreSample3.build();


        // parcare

        Action actParcare = parcheaza.build();

        // actiuni executate

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

                        actToSample2,
                        actPrepareSample2,

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

                        // specimen 3
                        new ParallelAction(
                                gheara.lasareGheara(),
                                actPrepareToTakeSample2,
                                glisiera.glisieraJos()
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

                        //specimen 4
                        new ParallelAction(
                                gheara.lasareGheara(),
                                actPrepareToTakeSample3,
                                glisiera.glisieraJos()
                        )
                )
        );
    }
}
