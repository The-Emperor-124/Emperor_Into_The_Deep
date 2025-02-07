package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.others.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "AlbastruCos", group = "Autonomous")
public class  AlbastruCos extends LinearOpMode{
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

        public class caramidaperete implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                servoGrDr.setPosition(0.7);
                servoGrSta.setPosition(0.2);
                return false;
            }
        }
        public Action peretecaramida(){return new caramidaperete();}

        public class RidicareGhearaBrat implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo_tg1.setPosition(0.52);
                servo_tg2.setPosition(0.52);
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

        public Action glisieraBara() {
            return GlisieraAction(1380, -1379, 0.8);
        }

        public Action glisieraSusJum() {
            return GlisieraAction(604, -608, 0.6);
        }

        public Action glisieraSusCos() {
            return GlisieraAction(3753, -3765, 0.7);    // n avea dely pe 0.7
        }   //  0.8 //0.6 inainte

        public Action glisieraSusCos12() {
            return GlisieraAction(3753, -3765, 0.9);
        }   //  0.8 //0.6 inainte

        public Action josGlisi() {
            return GlisieraAction(1, -1, -1.0);
        }       //      -0.9    //-0.6

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
        AccelConstraint accslow=new ProfileAccelConstraint(-45,90);
        AccelConstraint accfast=new ProfileAccelConstraint(-100,140);

        // asta e constrangerea de viteza pentru prima actiune
        VelConstraint p11 = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140.0)
        ));


        // prima data cand ajunge la cos
        TrajectoryActionBuilder p1 = drive.actionBuilder(pose)
                .strafeToConstantHeading(new Vector2d(15,68))
                .strafeToConstantHeading(new Vector2d(67.5, 63.5), p11, accslow) ;  // era 66 cand lovea 60.3, 63.2



        // rotire cos 1
        TrajectoryActionBuilder p2=p1.endTrajectory().fresh()
                .turnTo(358.8);

        TrajectoryActionBuilder p25=p2.endTrajectory().fresh()
                .lineToX(67.5);     // 67, 66.7


        // da inapoi de la cos
        TrajectoryActionBuilder p26=p2.endTrajectory().fresh()
                .lineToX(61.5);

        // noul vel constraint
        VelConstraint p33= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        // merge sa ia primul sample -> il lua bine
        TrajectoryActionBuilder p3=p26.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(55,53),p33,accfast);      // x : 54    // x : 54, y = 51

        // aici se intoarce si merge sa puna primul sample de pe jos(2)
        TrajectoryActionBuilder p4=p3.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(68,68));// x: 62.2//60.5 y

        TrajectoryActionBuilder p5=p4.endTrajectory().fresh()
                .lineToX(67.9);

        // asta da inapoi
        TrajectoryActionBuilder p6=p5.endTrajectory().fresh()
                .lineToX(61.5);

        // se roteste
        TrajectoryActionBuilder p7=p6.endTrajectory().fresh()
                .turnTo(-89.5);

        VelConstraint p88= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140.0),
                new AngularVelConstraint(Math.PI / 2)
        ));


        // asta e al doilea sample
        TrajectoryActionBuilder p8 = p7.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(66,53));      // x : 65.2, y : 52.2

        VelConstraint p99= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        // merge la cos sa puna al doilea sample
        TrajectoryActionBuilder p9 = p8.endTrajectory().fresh()
                .turnTo(88.4)
                .strafeToConstantHeading(new Vector2d(67.5, 66), p99,accfast);  // y : 61.2  //x : 60.3

        TrajectoryActionBuilder p10 = p9.endTrajectory().fresh()
                .turnTo(88.4);

        TrajectoryActionBuilder p101 = p10.endTrajectory().fresh()
                .lineToX(67);    // 66.7

        TrajectoryActionBuilder p102 = p10.endTrajectory().fresh()
                .lineToX(61.5);

        TrajectoryActionBuilder prob=p102.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(63, 46.4), p99,accfast) // 43 / x: 62 // 66 53  // 63 45
                .turnTo(-44.75);
        //                .splineToConstantHeading(
        //                        new Vector2d(60,44),Math.toRadians(0) // 63 cu 45 mergeau bine de pe initial pose //65, 46.5
        //                )

        TrajectoryActionBuilder prev12 = prob.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(50, 42), p99, accfast);   //y : 43

        TrajectoryActionBuilder p12 = prev12.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(67.5, 66), p99,accfast)
                .turnTo(88.4); // 88.4

        AccelConstraint accpark=new ProfileAccelConstraint(-90,190);

        TrajectoryActionBuilder p13 = p12.endTrajectory().fresh()
                //.lineToX(60.5); //61.5
                .strafeToConstantHeading(new Vector2d(57,66), p99, accpark);

        // Execute some actions before starting the trajectory
        Actions.runBlocking(gheara.ridicareGhearaBrat());
        Actions.runBlocking(gheara.prindereGheara());
        //Actions.runBlocking(gheara.peretecaramida());

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
        Action act7= p7.build();
        Action act8= p8.build();
        Action act9= p9.build();
        Action act10 = p10.build();
        Action act101 = p101.build();
        Action act102 = p102.build();
        Action act11=prob.build();

        Action act12 = p12.build();
        Action act13 = p13.build();

        Action actprev12 = prev12.build();

        // aici se vor executa toate actiunile

        Actions.runBlocking(
                // aici vor rula mai multe sequence action
            new SequentialAction(
                   new ParallelAction(
                            act1,
                            glisiera.glisieraSusCos12()
                           //act25
                    ),
                    act2,
                  // act25,

                    gheara.lasareGheara(),
                    act26,
                    new ParallelAction(
                            act3,
                            glisiera.josGlisi()
                    ),
                    gheara.pozitieObligaGheara(),
                    new SleepAction(0.4),
                    gheara.prindereGheara(),
                    new SleepAction(0.3),
                    gheara.ridicareGhearaBrat(),

                    new ParallelAction(
                            act4,
                            glisiera.glisieraSusCos12()
                    ),
                    //act5,
                    //new SleepAction(0.1), //la 0.2 era delay prea mare
                    gheara.lasareGheara(),
//                    new SleepAction(0.1),
                    act6,
                    act7,
                    new ParallelAction(
                            act8,
                            glisiera.josGlisi()
                    ),

                    gheara.pozitieObligaGheara(),
                    new SleepAction(0.4),
                    gheara.prindereGheara(),
                    new SleepAction(0.3),
                    gheara.ridicareGhearaBrat(),
                    new ParallelAction(
                            act9,
                            glisiera.glisieraSusCos()
                    ),
                   // act10,
                    //act101,
                    gheara.peretecaramida(),
                    act102,
                    //new SleepAction(0.1),
                    new ParallelAction(
                            act11,
                            glisiera.josGlisi()
                    ),
                    gheara.pozitieObligaGheara(),
                    new SleepAction(0.5),
                    gheara.prindereGheara(),
                    new SleepAction(0.3),
//                    gheara.ridicareGhearaBrat(),
                    actprev12,
                    new ParallelAction(
                            act12,
                            glisiera.glisieraSusCos12(),
                            gheara.ridicareGhearaBrat()
                    ),
                    gheara.lasareGheara(),
                    act13

            )
        );
    }
}