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

@Autonomous(name="AutoCosAlbastru", group = "Autonomous")
public class AutoCosAlbastru extends LinearOpMode {

    public class Gheara {
        private Servo servoGrDr, servoGrSta, servo_tg1, servo_tg2;

        public Gheara(HardwareMap hardwareMap) {
            servoGrDr = hardwareMap.get(Servo.class, "servoGrDr");
            servoGrSta = hardwareMap.get(Servo.class, "servoGrSta");
            servo_tg1 = hardwareMap.servo.get("servo_tg_stanga");
            servo_tg2 = hardwareMap.servo.get("servo_tg_dreapta");
            servo_tg2.setDirection(Servo.Direction.REVERSE);
            servoGrSta.setDirection(Servo.Direction.REVERSE);
            servoGrDr.setDirection(Servo.Direction.REVERSE);
        }

        public class PrindereGhera implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoGrDr.setPosition(0.97);
                servoGrSta.setPosition(-0.89);
                return false;
            }
        }

        public Action prindereGhera() {
            return new PrindereGhera();
        }

        public class LasareGheara implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoGrDr.setPosition(0.78);
                servoGrSta.setPosition(0.28);
                return false;
            }
        }

        public Action lasareGheara() {
            return new LasareGheara();
        }

        public class ridicare_gheara_brat implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo_tg1.setPosition(0.55);
                servo_tg2.setPosition(0.55);
                return false;
            }
        }

        public Action ridicare_gheara_brat() {
            return new ridicare_gheara_brat();
        }
        public class pozitie_obliga_gheara implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo_tg1.setPosition(0.9);
                servo_tg2.setPosition(0.9);
                return false;
            }
        }
        public Action pozitie_obliga_gheara() {
            return new pozitie_obliga_gheara();
        }

    }

    public class Glisiera {
        private DcMotor motor_brat1, motor_brat2;

        public Glisiera(HardwareMap hardwareMap) {
            motor_brat1 = hardwareMap.dcMotor.get("motor_brat1");
            motor_brat2 = hardwareMap.dcMotor.get("motor_brat2");
            motor_brat1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_brat2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class Sus_Glisi implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Add logging to make sure the motor is running
                telemetry.addData("Glisiera", "Attempting to move up");
                telemetry.update();

                // Test if motors are responding
                motor_brat1.setPower(0.5); // Set a fixed power to the motor
                motor_brat2.setPower(0.5);

                // Set target positions for the motors (these should be reasonable values based on your system)
                motor_brat1.setTargetPosition(2259);
                motor_brat2.setTargetPosition(-2254);

                motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Log the motor target positions
                telemetry.addData("Motor 1 Target", motor_brat1.getTargetPosition());
                telemetry.addData("Motor 2 Target", motor_brat2.getTargetPosition());
                telemetry.update();

                // Wait until the motors reach their target positions
                while (opModeIsActive() && motor_brat1.isBusy() && motor_brat2.isBusy()) {
                    telemetry.addData("Motor 1", "Position: " + motor_brat1.getCurrentPosition());
                    telemetry.addData("Motor 2", "Position: " + motor_brat2.getCurrentPosition());
                    telemetry.update();
                }

                // Stop motors after reaching the target
                motor_brat1.setPower(0.11);
                motor_brat2.setPower(-0.11);

                telemetry.addData("Glisiera", "Movement complete");
                telemetry.update();
                return false;
            }
        }


        public Action GlisieraSus() {
            return new Sus_Glisi();
        }

        public class Sus_Glisi_Jum implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Add logging to make sure the motor is running
                telemetry.addData("Glisiera", "Attempting to move up");
                telemetry.update();

                // Test if motors are responding
                motor_brat1.setPower(0.5); // Set a fixed power to the motor
                motor_brat2.setPower(0.5);

                // Set target positions for the motors (these should be reasonable values based on your system)
                motor_brat1.setTargetPosition(604);
                motor_brat2.setTargetPosition(-608);

                motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Log the motor target positions
                telemetry.addData("Motor 1 Target", motor_brat1.getTargetPosition());
                telemetry.addData("Motor 2 Target", motor_brat2.getTargetPosition());
                telemetry.update();

                // Wait until the motors reach their target positions
                while (opModeIsActive() && motor_brat1.isBusy() && motor_brat2.isBusy()) {
                    telemetry.addData("Motor 1", "Position: " + motor_brat1.getCurrentPosition());
                    telemetry.addData("Motor 2", "Position: " + motor_brat2.getCurrentPosition());
                    telemetry.update();
                }
                motor_brat1.getCurrentPosition();
                motor_brat2.getCurrentPosition();

                // Stop motors after reaching the target
                motor_brat1.setPower(0.11);
                motor_brat2.setPower(-0.11);

                telemetry.addData("Glisiera", "Movement complete");
                telemetry.update();
                return false;
            }
        }

        public Action GlisieraSusJum() {
            return new Sus_Glisi_Jum();
        }

       /* public class Sus_Glisi_cos implements Action {
            private boolean isstarte=false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!isstarte) {
                    // Add logging to make sure the motor is running
                    telemetry.addData("Glisiera", "Attempting to move up");
                    telemetry.update();

                    // Test if motors are responding
                    motor_brat1.setPower(0.6); // Set a fixed power to the motor
                    motor_brat2.setPower(0.6);

                    // Set target positions for the motors (these should be reasonable values based on your system)
                    motor_brat1.setTargetPosition(3753);
                    motor_brat2.setTargetPosition(-3765);

                    motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Log the motor target positions
                    telemetry.addData("Motor 1 Target", motor_brat1.getTargetPosition());
                    telemetry.addData("Motor 2 Target", motor_brat2.getTargetPosition());
                    telemetry.update();

                    // Wait until the motors reach their target positions
                    while (opModeIsActive() && motor_brat1.isBusy() && motor_brat2.isBusy()) {
                        telemetry.addData("Motor 1", "Position: " + motor_brat1.getCurrentPosition());
                        telemetry.addData("Motor 2", "Position: " + motor_brat2.getCurrentPosition());
                        telemetry.update();
                    }
                    motor_brat1.getCurrentPosition();
                    motor_brat2.getCurrentPosition();
                }
                if(motor_brat1.isBusy() && motor_brat2.isBusy()) { return false;}
else{
                    // Stop motors after reaching the target
                    motor_brat1.setPower(0.11);
                    motor_brat2.setPower(-0.11);


                telemetry.addData("Glisiera", "Movement complete");
                telemetry.update();
                return false;
                }
            }


        }*/

        public class Sus_Glisi_cos implements Action {
            private boolean isStarted = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isStarted) {
                    motor_brat1.setPower(0.6);
                    motor_brat2.setPower(0.6);
                    motor_brat1.setTargetPosition(3753);
                    motor_brat2.setTargetPosition(-3765);
                    motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    isStarted = true;
                }

                // Check if the motors are still busy; if yes, keep running
                if (motor_brat1.isBusy() || motor_brat2.isBusy()) {
                    return true;  // Returning true means the action is still ongoing
                } else {
                    motor_brat1.setPower(0.11);
                    motor_brat2.setPower(-0.11);
                    return false;  // Returning false means the action is complete
                }
            }
        }


        public Action GlisieraSusCos() {
            return new Sus_Glisi_cos();
        }

        public class Jos_Glisi implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Add logging to make sure the motor is running
                telemetry.addData("Glisiera", "Attempting to move up");
                telemetry.update();

                // Test if motors are responding
                motor_brat1.setPower(-0.6); // Set a fixed power to the motor
                motor_brat2.setPower(-0.6);

                // Set target positions for the motors (these should be reasonable values based on your system)
                motor_brat1.setTargetPosition(1);
                motor_brat2.setTargetPosition(-1);

                motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Log the motor target positions
                telemetry.addData("Motor 1 Target", motor_brat1.getTargetPosition());
                telemetry.addData("Motor 2 Target", motor_brat2.getTargetPosition());
                telemetry.update();

                // Wait until the motors reach their target positions
                while (opModeIsActive() && motor_brat1.isBusy() && motor_brat2.isBusy()) {
                    telemetry.addData("Motor 1", "Position: " + motor_brat1.getCurrentPosition());
                    telemetry.addData("Motor 2", "Position: " + motor_brat2.getCurrentPosition());
                    telemetry.update();
                }
                motor_brat1.getCurrentPosition();
                motor_brat2.getCurrentPosition();

                // Stop motors after reaching the target
                motor_brat1.setPower(0.11);
                motor_brat2.setPower(-0.11);

                telemetry.addData("Glisiera", "Movement complete");
                telemetry.update();
                return false;
            }


        }
        public Action JosGlisi() {
            return new Jos_Glisi();
        }
        public class Glisi_Parcare implements Action {
            private  boolean isstarted=false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!isstarted) {
                    // Add logging to make sure the motor is running
                    telemetry.addData("Glisiera", "Attempting to move up");
                    telemetry.update();

                    // Test if motors are responding
                    motor_brat1.setPower(-0.6); // Set a fixed power to the motor
                    motor_brat2.setPower(-0.6);

                    // Set target positions for the motors (these should be reasonable values based on your system)
                    motor_brat1.setTargetPosition(447);
                    motor_brat2.setTargetPosition(-444);

                    motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    isstarted=true;


                // Log the motor target positions
                telemetry.addData("Motor 1 Target", motor_brat1.getTargetPosition());
                telemetry.addData("Motor 2 Target", motor_brat2.getTargetPosition());
                telemetry.update();

                // Wait until the motors reach their target positions
                while (opModeIsActive() && motor_brat1.isBusy() && motor_brat2.isBusy()) {
                    telemetry.addData("Motor 1", "Position: " + motor_brat1.getCurrentPosition());
                    telemetry.addData("Motor 2", "Position: " + motor_brat2.getCurrentPosition());
                    telemetry.update();
                }
                motor_brat1.getCurrentPosition();
                motor_brat2.getCurrentPosition();
                }
                if(motor_brat1.isBusy() && motor_brat2.isBusy()){
                    return  true;}
                else{
                    // Stop motors after reaching the target
                    motor_brat1.setPower(0.11);
                    motor_brat2.setPower(-0.11);

                    telemetry.addData("Glisiera", "Movement complete");
                    telemetry.update();
                    return false;
                }
            }


        }
        public Action parcareGlisi() {
            return new Glisi_Parcare();
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


    @Override
    public void runOpMode() {
        // Instantiate Gheara and Glisiera objects
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        // Initialize the robot's position (for MecanumDrive)
        Pose2d pose = new Pose2d(17.8, 70.1, -89.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        // Create a trajectory action
        VelConstraint p11= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(70.0)

        ));
        TrajectoryActionBuilder p1 = drive.actionBuilder(pose)
                //.strafeToConstantHeading(new Vector2d(60.3,61.2));
                .strafeToConstantHeading(new Vector2d(60.3,61.2),p11);

        //.lineToY(-0.006);
        TrajectoryActionBuilder p2=p1.endTrajectory().fresh()
                //.strafeToConstantHeading(new Vector2d(-68.9,69.1));
                .turnTo(358.8);
        TrajectoryActionBuilder p25=p2.endTrajectory().fresh()
                .lineToX(66.7);
        TrajectoryActionBuilder p26=p25.endTrajectory().fresh()
                .lineToX(61.5);
        VelConstraint p33= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        TrajectoryActionBuilder p3=p26.endTrajectory().fresh()
                .turnTo(-89.5)
                .strafeToConstantHeading(new Vector2d(54,51),p33);
        //.strafeToConstantHeading(new Vector2d(54,51.5));


       /* .strafeToConstantHeading(new Vector2d(44.1,56.2))
                .strafeToConstantHeading(new Vector2d(44.1,-10))
         .turnTo(179.1)
        .lineToX(28);*/

        TrajectoryActionBuilder p4=p3.endTrajectory().fresh()
                .turnTo(88.4)
        .strafeToConstantHeading(new Vector2d(62.2,60.5));
        TrajectoryActionBuilder p5=p4.endTrajectory().fresh()
                .lineToX(67.9);
        TrajectoryActionBuilder p6=p5.endTrajectory().fresh()
                .lineToX(61.5);


        Action trajectoryActionCloseOut = p6.endTrajectory().fresh()
                .build();

        // Execute some actions before starting the trajectory
        Actions.runBlocking(gheara.ridicare_gheara_brat());
        Actions.runBlocking(gheara.prindereGhera());



        // Wait for the start signal
        waitForStart();

        // Execute the planned actions after the start
        Actions.runBlocking(
                /*
                new SequentialAction(
                        // Perform the trajectory

                        p1.build(),
                        new ParallelAction(
                                p2.build(),
                                        //new SleepAction(0.02),
                                        glisiera.GlisieraSusCos()

                        ),
                        new SleepAction(0.5),
                        p25.build(),
                        gheara.lasareGheara(),
                        gheara.ridicare_gheara_brat(),
                        p26.build(),
                        glisiera.JosGlisi(),
                        //nou
                       p3.build(),
                        gheara.pozitie_obliga_gheara(),
                        new SleepAction(0.6),
                        gheara.prindereGhera(),
                        new SleepAction(0.4),
                        gheara.ridicare_gheara_brat(),
                        new SleepAction(0.5),

                                // aici decomenteaza daca nu merge
                                new ParallelAction(
                                        p4.build(),
                                        glisiera.GlisieraSusCos()
                                ),



                        new ParallelAction(
                                p4.build(),
                                glisiera.GlisieraSusCos()
                        ),
                        new SleepAction(0.5),
                        p5.build(),
                        gheara.lasareGheara(),
                        p6.build(),
                        glisiera.JosGlisi(),
                        trajectoryActionCloseOut
                      //glisiera.parcareGlisi(),
                     //glisiera.parcareGlisi(),




                )*/

                new SequentialAction(
                        // Initial trajectory and parallel movement of glisiera
                        p1.build(),
                        new ParallelAction(
                                p2.build(),
                                glisiera.GlisieraSusCos()
                        ),

                        // Move to position and operate gheara
                        p25.build(),
                        new ParallelAction(
                                gheara.lasareGheara(),
                                gheara.ridicare_gheara_brat()
                        ),

                        // Continue movement while adjusting glisiera
                        new ParallelAction(
                                p26.build(),
                                glisiera.JosGlisi()
                        ),

                        // Move to next position and prepare gheara
                        p3.build(),
                        gheara.pozitie_obliga_gheara(),

                        // Sleep for synchronization, then grip and lift with minimal delays
                        new SequentialAction(
                                new SleepAction(0.6),
                                gheara.prindereGhera(),
                                new SleepAction(0.4),
                                gheara.ridicare_gheara_brat(),
                                new SleepAction(0.5)
                        ),

                        // Move and lift simultaneously
                        new ParallelAction(
                                p4.build(),
                                glisiera.GlisieraSusCos()
                        ),

                        // Move to drop-off and release object
                        p5.build(),
                        gheara.lasareGheara(),

                        // Move and lower glisiera simultaneously
                        new ParallelAction(
                                p6.build(),
                                glisiera.JosGlisi()
                        ),

                        // Final trajectory to close out
                        trajectoryActionCloseOut
                )
        );
    }
}
