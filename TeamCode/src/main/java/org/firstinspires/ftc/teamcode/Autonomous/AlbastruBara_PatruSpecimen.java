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

        VelConstraint speedFast= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(130.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        VelConstraint speedSlow= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        //prima actiune este cea in care mergem la bara sa punem preload
        TrajectoryActionBuilder drumPreload = drive.actionBuilder(pose)
                .strafeToConstantHeading(new Vector2d(-0, 33), speedFast, accFast);    // are 37 // -40

        TrajectoryActionBuilder plecareDupaPreload=drumPreload.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-46, 45), speedFast, accFast);


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



        // actiuni executate

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(
                                actPreload,
                                glisiera.glisieraSusJumate()
                        ),

                        glisiera.glisieraOutake(),
                        // aici lasare gheara
                        actPlecareDupaPreload
                )
        );
    }
}
