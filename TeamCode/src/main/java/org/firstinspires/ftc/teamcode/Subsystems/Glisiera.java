package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    // asta e la modu general
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

    public Action glisieraSus(){
        return GlisieraAction(2259, -2254, 0.6);
    }

    public Action glisieraSusJumate(){
        return GlisieraAction(1182, -1207, 1.0);
    }

    public Action glisieraOutake(){
        return GlisieraAction(400, -400, -1.0);
    }

    public Action glisieraJos(){
        return GlisieraAction(1, -1, -1.0);
    } //erau 1 -1 -1.0

    public Action glisieraParcareBara() {
        return GlisieraAction(1474, -1470, 0.8);
    } // erau 1380 -1379

    public Action glisieraSusCos() {
        return GlisieraAction(3853, -3865, 0.7);    // n avea delay pe 0.7
    }

    public Action glisieraSusCosRapid() {
        return GlisieraAction(3468, -3456, 0.9); // erau 3853 -38665 0.9
    }   //  0.8 //0.6 inainte
}