package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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

@Autonomous(name = "AlbastruCos", group = "Autonomous")
public class AlbastruCos extends LinearOpMode{
    public class Gheara{
        private Servo servoGrDr, servoGrSta, servo_tg1, servo_tg2;

        public Gheara(HardwareMap hardwareMap){
            servoGrDr = hardwareMap.get(Servo.class, "servoGrDr");
            servoGrSta = hardwareMap.get(Servo.class, "servoGrSta");
            servo_tg1 = hardwareMap.servo.get("servo_tg_stanga");
            servo_tg2 = hardwareMap.servo.get("servo_tg_dreapta");
            servo_tg2.setDirection(Servo.Direction.REVERSE);
            servoGrSta.setDirection(Servo.Direction.REVERSE);
            servoGrDr.setDirection(Servo.Direction.REVERSE);
        }

        // actiunea de a strange gheara

        public class PrindereGheara implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                servoGrDr.setPosition(0.97);
                servoGrSta.setPosition(-0.89);
                return false;
            }
        }

        // metoda
        public Action prindereGheara(){
            return new PrindereGheara();
        }

        public class LasareGheara implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                servoGrDr.setPosition(0.78);
                servoGrSta.setPosition(0.28);
                return false;
            }
        }

        public Action lasareGheara() {
            return new LasareGheara();
        }

        public class RidicareGhearaBrat implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo_tg1.setPosition(0.55);
                servo_tg2.setPosition(0.55);
                return false;
            }
        }

        public Action ridicareGhearaBrat(){
            return new RidicareGhearaBrat();
        }

        public class PozitieObligaGheara implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                servo_tg1.setPosition(0.9);
                servo_tg2.setPosition(0.9);
                return false;
            }
        }

        public Action pozitieObligaGheara(){
            return new PozitieObligaGheara();
        }

    }



    // glisiera actiuni

    public class Glisiera{
        private DcMotor motor_brat1, motor_brat2;

        // declarari si legaturi motoare
        public Glisiera(HardwareMap hardwareMap) {
            motor_brat1 = hardwareMap.dcMotor.get("motor_brat1");
            motor_brat2 = hardwareMap.dcMotor.get("motor_brat2");
            motor_brat1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_brat2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private Action GlisieraAction(int position1, int position2, double power){
            //TelemetryPacket packet;
            return packet->{
                motor_brat1.setTargetPosition(position1);
                motor_brat2.setTargetPosition(position2);
                motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat1.setPower(power);
                motor_brat2.setPower(power);

                if (motor_brat1.isBusy() || motor_brat2.isBusy()) {
                    return true;
                } else {
                    motor_brat1.setPower(0.1);
                    motor_brat2.setPower(-0.1);
                    return false;
                }
            };

        }

        // actiuni

        public Action glisieraSus() {
            return GlisieraAction(2259, -2254, 0.5);
        }

        public Action glisieraSusJum() {
            return GlisieraAction(604, -608, 0.5);
        }

        public Action glisieraSusCos() {
            return GlisieraAction(3753, -3765, 0.6);
        }

        public Action josGlisi() {
            return GlisieraAction(1, -1, -0.6);
        }

        public Action parcareGlisi() {
            return GlisieraAction(447, -444, -0.6);
        }


        public class PutereConst implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                motor_brat1.setPower(0.01);
                motor_brat2.setPower(-0.01);

                telemetry.addData("Glisiera", "Movement complete");
                telemetry.update();
                return false;
            }


        }
        public Action putconst() {
            return new PutereConst();
        }
    }


    // aici se petrece de fapt tot

    @Override
    public void runOpMode(){
        // instantiem Gheara si Glisiera -> aka subsistemele robotului
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        // Initialize the robot's position (for MecanumDrive)
        Pose2d pose = new Pose2d(17.8, 70.1, -89.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        // asta e constrangerea de viteza pentru prima actiune
        VelConstraint p11 = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(70.0)
        ));

        TrajectoryActionBuilder p1 = drive.actionBuilder(pose)
                .strafeToConstantHeading(new Vector2d(60.3, 61.2), p11);


        TrajectoryActionBuilder p2=p1.endTrajectory().fresh()
                .turnTo(358.8);

        TrajectoryActionBuilder p25=p2.endTrajectory().fresh()
                .lineToX(66.7);

        TrajectoryActionBuilder p26=p25.endTrajectory().fresh()
                .lineToX(61.5);

        // noul vel constraint
        VelConstraint p33= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        TrajectoryActionBuilder p3=p26.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(54,51),p33);

        TrajectoryActionBuilder p4=p3.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(62.2,60.5));

        TrajectoryActionBuilder p5=p4.endTrajectory().fresh()
                .lineToX(67.9);

        TrajectoryActionBuilder p6=p5.endTrajectory().fresh()
                .lineToX(61.5);

        // Execute some actions before starting the trajectory
        Actions.runBlocking(gheara.ridicareGhearaBrat());
        Actions.runBlocking(gheara.prindereGheara());

        // Wait for the start signal
        waitForStart();

        if(isStopRequested()){
            return;
        }

        // aici compilam actiunile ca sa nu mai piarda timp
        Action act1 = p1.build();
        Action act2 = p2.build();
        Action act25 = p25.build();
        Action act26 = p26.build();
        Action act3 = p3.build();
        Action act4 = p4.build();
        Action act5 = p5.build();
        Action act6 = p6.build();

        // aici se vor executa toate actiunile

        Actions.runBlocking(
                // aici vor rula mai multe sequence action
            new SequentialAction(

            )
        );
    }
}