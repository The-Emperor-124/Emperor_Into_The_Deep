package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Mecanum_AnalogSticks2025_Encodere", group="Linear Opmode")
//@Disabled
public class Mecanum_AnalogSticks2025_Encodere extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftRear, leftFront, rightRear, rightFront, motor_brat1,motor_brat2;

    Servo servo_tg1, servo_tg2,servoGrDr,servoGrSta;

    double power1,power2,posi1,posi2;
    public void pos1(int position,double power1)
    {motor_brat1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat1.setTargetPosition(position);
        motor_brat1.setPower(power1);
        motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void pos2(int position,double power2)
    {
        motor_brat2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat2.setTargetPosition(position);
        motor_brat2.setPower(power2);
        motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() {

        leftRear            = hardwareMap.dcMotor.get("leftRear");
        leftFront           = hardwareMap.dcMotor.get("leftFront");
        rightRear           = hardwareMap.dcMotor.get("rightRear");
        rightFront          = hardwareMap.dcMotor.get("rightFront");

        servo_tg1=hardwareMap.servo.get("servo_tg_stanga");
        servo_tg2=hardwareMap.servo.get("servo_tg_dreapta");
        servoGrDr=hardwareMap.servo.get("servoGrDr");
        servoGrSta=hardwareMap.servo.get("servoGrSta");
        motor_brat1 =hardwareMap.dcMotor.get("motor_brat1");
        motor_brat2=hardwareMap.dcMotor.get("motor_brat2");


        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        servo_tg2.setDirection(Servo.Direction.REVERSE);
        servoGrSta.setDirection(Servo.Direction.REVERSE);
        servoGrDr.setDirection(Servo.Direction.REVERSE);
        servo_tg2.setPosition(0.5);// era 0.01
        servo_tg1.setPosition(0.5);// era 0.01
        servoGrDr.setPosition(0.95);//-0.3 si 0
        servoGrSta.setPosition(-1); //era 0.33 dupa 0.97
        motor_brat1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double power;

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("brat1:",motor_brat1.getCurrentPosition());
            telemetry.addData("brat2:",motor_brat2.getCurrentPosition());
            telemetry.update();


            //turbo slow si normal
            if(gamepad1.left_trigger>0.0)
            {//                         fata/spate                 strafe                          rotire
                power=0.2;
                leftRear.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                rightRear.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 + gamepad1.right_stick_x) * power);
            }
            else if(gamepad1.right_trigger>0.0)
            {
                power=0.9;
                leftRear.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                rightRear.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 + gamepad1.right_stick_x) * power);
            }
            else
            {
                power=0.6;
                leftRear.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                rightRear.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x / 0.85 - gamepad1.right_stick_x) * power);
                rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x / 0.85 + gamepad1.right_stick_x) * power);
            }

            telemetry.update();

            if(gamepad2.right_bumper)
            {
                //motor_brat1.setPower(0);//motor_brat2.setPower(0);
                servo_tg1.setPosition(0.69);
                servo_tg2.setPosition(0.69);
            }
            if(gamepad2.left_bumper)
            {
                servo_tg1.setPosition(0.83);
                servo_tg2.setPosition(0.83);
            }
            if(gamepad2.dpad_up)
            {
                servo_tg1.setPosition(0.55);
                servo_tg2.setPosition(0.55);
            }
            if(gamepad2.dpad_left)
            {
                servo_tg2.setPosition(0.65);
                servo_tg1.setPosition(0.65);
            }
            if(gamepad2.dpad_right)
            {
                servo_tg1.setPosition(0.78);
                servo_tg2.setPosition(0.78);
            }
            if(gamepad2.dpad_down)
            {
                servo_tg1.setPosition(0.84);
                servo_tg2.setPosition(0.84);
            }
            if(gamepad2.y)
            {
                servoGrSta.setPosition(0.24);
                servoGrDr.setPosition(0.80);
            }
            if(gamepad2.a) //prindere
            {
                servoGrSta.setPosition(-0.97);//-0.97
                servoGrDr.setPosition(0.96);//0.97   dupa 0.96
            }
            if (gamepad2.left_trigger>0.0)
            {   motor_brat1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_brat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                posi1=motor_brat1.getCurrentPosition();
                posi2=motor_brat2.getCurrentPosition();
                motor_brat1.setPower(-0.87);
                motor_brat2.setPower(1.0);

            }
            else
            { if(posi1<1.0 && posi2<1.0) { motor_brat1.setPower(0); motor_brat2.setPower(0);}
                motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_brat1.setTargetPosition((int)(posi1));
                motor_brat2.setTargetPosition((int)(posi2));
                //motor_brat1.setPower(-0.1);
                // motor_brat2.setPower(0.1);
                motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if (gamepad2.right_trigger>0.0)
            {motor_brat1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_brat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                posi1=motor_brat1.getCurrentPosition();
                posi2=motor_brat2.getCurrentPosition();
                motor_brat1.setPower(1.0);
                motor_brat2.setPower(-1.0);


            }
            else
            {   //if(posi1<1.0 && posi2<1.0) { motor_brat1.setPower(0); motor_brat2.setPower(0);}
                motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_brat1.setTargetPosition((int)(posi1));
                motor_brat2.setTargetPosition((int)(posi2));
                motor_brat1.setPower(0.01);
                motor_brat2.setPower(-0.01);
                motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }




    }
}

