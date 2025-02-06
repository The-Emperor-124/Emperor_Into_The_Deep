package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutoCosRosu", group = "Autonomous")
@Disabled
public class AutoCosRosu extends LinearOpMode {

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
                servoGrDr.setPosition(0.80);
                servoGrSta.setPosition(0.24);
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
                servo_tg1.setPosition(0.70);
                servo_tg2.setPosition(0.70);
                return false;
            }
        }
        public Action pozitie_jos() {
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

        public class Sus_Glisi_cos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Add logging to make sure the motor is running
                telemetry.addData("Glisiera", "Attempting to move up");
                telemetry.update();

                // Test if motors are responding
                motor_brat1.setPower(0.5); // Set a fixed power to the motor
                motor_brat2.setPower(0.5);

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

                // Stop motors after reaching the target
                motor_brat1.setPower(0.11);
                motor_brat2.setPower(-0.11);

                telemetry.addData("Glisiera", "Movement complete");
                telemetry.update();
                return false;
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
                motor_brat1.setPower(-0.5); // Set a fixed power to the motor
                motor_brat2.setPower(-0.5);

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
    }


    @Override
    public void runOpMode() {
        // Instantiate Gheara and Glisiera objects
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        // Initialize the robot's position (for MecanumDrive)
        Pose2d pose = new Pose2d(-17.8, -70.1, 89.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        // Create a trajectory action
        TrajectoryActionBuilder p1 = drive.actionBuilder(pose)
                .strafeToConstantHeading(new Vector2d(-61,-58.9));

        //.lineToY(-0.006);
        TrajectoryActionBuilder p2=p1.endTrajectory().fresh()
                //.strafeToConstantHeading(new Vector2d(-68.9,69.1));
                .turnTo(179.7);
        TrajectoryActionBuilder p25=p2.endTrajectory().fresh()
                .lineToX(-64.2);
        TrajectoryActionBuilder p26=p25.endTrajectory().fresh()
                .lineToX(-54.9);
        //TrajectoryActionBuilder p3=p26.endTrajectory().fresh()
          //      .strafeTo(new Vector2d(-23.8,-69.4));

        TrajectoryActionBuilder p4=p26.endTrajectory().fresh()
                .turnTo(89.5);
        TrajectoryActionBuilder p5=p4.endTrajectory().fresh()
                .lineToX(-48);
         TrajectoryActionBuilder p6=p5.endTrajectory().fresh()
                 .lineToX(-54.9);
         TrajectoryActionBuilder p7=p6.endTrajectory().fresh()
                 .turnTo(179.8);//90.69
        TrajectoryActionBuilder p8=p7.endTrajectory().fresh()
                .lineToX(-62.4);

        Action trajectoryActionCloseOut = p8.endTrajectory().fresh()
                .build();

        // Execute some actions before starting the trajectory
        Actions.runBlocking(gheara.ridicare_gheara_brat());
        Actions.runBlocking(gheara.prindereGhera());

        // Wait for the start signal
        waitForStart();

        // Execute the planned actions after the start
        Actions.runBlocking(
                new SequentialAction(
                        // Perform the trajectory

                        p1.build(),
                        p2.build(),
                        glisiera.GlisieraSusCos(),
                        p25.build(),
                        gheara.lasareGheara(),
                        gheara.ridicare_gheara_brat(),
                        p26.build(),
                        glisiera.JosGlisi(),
                        //ce e nou

                       p4.build(),
                        p5.build(),
                        gheara.pozitie_jos(),
                        gheara.prindereGhera(),
                        gheara.ridicare_gheara_brat(),
                        p6.build(),
                        p7.build(),
                        glisiera.GlisieraSusCos(),
                        p8.build(),
                        gheara.lasareGheara()



                )
        );
    }
}
