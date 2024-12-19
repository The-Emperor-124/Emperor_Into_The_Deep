package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name="PrimaIcercareAuto",group = "Autonomous")
public class PrimaIcercareAuto extends LinearOpMode {
        //classes
    public  class Gheara{
        private Servo servoGrDr,servoGrSta;
        public Gheara(HardwareMap hardwareMap)
        {
            servoGrDr =hardwareMap.get(Servo.class,"servoGrDr");
                    servoGrSta=hardwareMap.get(Servo.class,"servoGrSta");
        }
        public class RidicareGhera implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                servoGrDr.setPosition(0.8);
                servoGrSta.setPosition(0.8);
                return  false;

            }
        }
        public Action RidicaGheara()
        {
            return  new RidicareGhera();
        }
        public class CoborareGheara implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                servoGrDr.setPosition(0.2);
                servoGrSta.setPosition(0.2);
                return  false;
            }
        }
        public  Action CoborareGheara()
        {
            return  new CoborareGheara();
        }
    }
    @Override
    public  void runOpMode()
    {
        Gheara Gheara=new Gheara(hardwareMap);
        Pose2d pose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive=new MecanumDrive(hardwareMap,pose);
        TrajectoryActionBuilder p1=drive.actionBuilder(pose)
          .lineToY(33)
                .waitSeconds(3)
                .setTangent(Math.toRadians(90));
        Action trajectoryActionCloseOut = p1.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 0))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        p1.build(),
                        Gheara.RidicaGheara(),
                        Gheara.CoborareGheara(),
                        trajectoryActionCloseOut

                )
        );
    }
}
