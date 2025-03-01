/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Reziduu;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@Autonomous(name="AlbastruSus2", group="Autonomous")
@Disabled

public class AlbastruSus2 extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftRear, leftFront, rightRear, rightFront, motor_brat1,motor_brat2;;

    Servo servo_tg1, servo_tg2,servoGrDr,servoGrSta;

    private ElapsedTime     runtime = new ElapsedTime();

    double posi1,posi2;

    static final double     FORWARD_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.5;
    public void fata()
    {
        leftRear.setPower(FORWARD_SPEED);
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        rightRear.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.7)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void strafe_left()
    {

        leftRear.setPower(TURN_SPEED);
        leftFront.setPower(-TURN_SPEED);
        rightFront.setPower(TURN_SPEED);
        rightRear.setPower(-TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.4)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public  void pune_caramida()
    {
        servoGrSta.setPosition(0.24);
        servoGrDr.setPosition(0.94);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.24)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void strafe_right()
    {
        leftRear.setPower(-TURN_SPEED);
        leftFront.setPower(TURN_SPEED);
        rightFront.setPower(-TURN_SPEED);
        rightRear.setPower(TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void ridica_glisiera()
    {
        motor_brat2.setPower(-0.8);
        motor_brat1.setPower(0.8);
        motor_brat1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        posi1=motor_brat1.getCurrentPosition();
        posi2=motor_brat2.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void coborare_glisiera()
    {

        motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat1.setTargetPosition((int)(posi1));
        motor_brat2.setTargetPosition((int)(posi2));
        //motor_brat1.setPower(-0.1);
        // motor_brat2.setPower(0.1);
        motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
      /*  while (opModeIsActive() && (runtime.seconds() <0.2)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
    }
    public void stopi()
    {
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    public void fata_mic(){
        leftRear.setPower(FORWARD_SPEED);
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        rightRear.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void spate()
    {
        leftRear.setPower(-FORWARD_SPEED);
        leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        rightRear.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void pozitie()
    {
        servo_tg1.setPosition(0.12);
        servo_tg2.setPosition(0.12);
    }
    public void rotire()
    {
        leftRear.setPower(FORWARD_SPEED);
        leftFront.setPower(FORWARD_SPEED);
        rightRear.setPower(-FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }
    void stopglisi()
    {
        motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat1.setTargetPosition((int)(posi1));
        motor_brat2.setTargetPosition((int)(posi2));
        motor_brat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void  rotstg()
    {
        leftRear.setPower(-FORWARD_SPEED);
        leftFront.setPower(-FORWARD_SPEED);
        rightRear.setPower(FORWARD_SPEED);
        leftRear.setPower(FORWARD_SPEED);
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
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
        motor_brat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        servo_tg2.setDirection(Servo.Direction.REVERSE);
        servoGrSta.setDirection(Servo.Direction.REVERSE);
        servo_tg2.setPosition(0.55);
        servo_tg1.setPosition(0.55);
        servoGrDr.setPosition(0.82);//0.85
        servoGrSta.setPosition(-0.87);//-0.89



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        // pozitie();
        //fata_mic();
        //rotstg();
        //fata();
        //ridica_glisiera();
        //stopglisi();
        //pune_caramida();
        //spate();
      //  coborare_glisiera();
       /* strafe_left();
        fata(); fata_mic(); fata_mic(); fata_mic();
        strafe_right();
        rotire();
        ridica_glisiera();
        rotire();
        ridica_glisiera();
        coborare_glisiera();
        sleep(50);
        //ridica_glisiera();

        //pune_caramida();
        //  coborare_glisiera();
        // spate();
       strafe_right();
       fata();
        strafe_left();*/
        //stopi();
        strafe_left();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
