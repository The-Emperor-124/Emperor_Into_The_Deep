package org.firstinspires.ftc.teamcode.Reziduu;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.others.MecanumDrive;

@Autonomous(name="AutoPeBaraRosu", group = "Autonomous")
@Disabled
public class AutoBaraRosu extends LinearOpMode {

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
        public class pozitie_fata_obliga implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo_tg1.setPosition(0.66);
                servo_tg2.setPosition(0.66);
                return false;
            }
        }
        public Action pozitie_obliga(){
            return new pozitie_fata_obliga();
        }
    }

    public class Glisiera {
        private DcMotor motor_brat1,motor_brat2;

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

    }


    @Override
    public void runOpMode() {
        // Instantiate Gheara and Glisiera objects
        Gheara gheara = new Gheara(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);

        // Initialize the robot's position (for MecanumDrive)
        Pose2d pose = new Pose2d(23.6, -70.3, 80.1);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        // Create a trajectory action
        TrajectoryActionBuilder p1 = drive.actionBuilder(pose)
                .splineToConstantHeading(new Vector2d(-6.2,45.4),Math.toRadians(0));

        //.lineToY(-0.006);
        TrajectoryActionBuilder p2=p1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-68.9,69.1));
        Action trajectoryActionCloseOut = p2.endTrajectory().fresh()
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
                        //glisiera.GlisieraSusJum(),
                        //p1.build(),

                        /*new ParallelAction(
                                gheara.pozitie_obliga(),
                                glisiera.GlisieraSus()

                        )*/
                        // new SleepAction(0.1)

                )
        );
    }
}
