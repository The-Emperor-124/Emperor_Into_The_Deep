package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import SubSystems.Gheara;
import SubSystems.Glisiera;


@Autonomous(name="RosuBara", group = "Autonomous")
public class RosuBara extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        Pose2d pose = new Pose2d(23.6, -70.3, 80.1);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);


        Actions.runBlocking(gheara.ridicareGhearaBrat());
        Actions.runBlocking(gheara.prindereGheara());
        Actions.runBlocking(glisiera.glisieraSusJumate());

        // Wait for the start signal
        waitForStart();


        if(isStopRequested()){
            return;
        }

        // aici compilam actiunile ca sa nu mai piarda timp


        //aici se executa toate actiunile

        Actions.runBlocking(
                new SequentialAction(
                )
        );

    }
}
