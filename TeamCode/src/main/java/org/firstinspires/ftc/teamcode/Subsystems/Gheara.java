package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gheara {
    private Servo servoGrDr, servoGrSta, servoTg1, servoTg2;

    // hardware map -> legatura cu hub ul si hardware ul
    public Gheara(HardwareMap hardwareMap) {

        servoGrDr = hardwareMap.get(Servo.class, "servoGrDr");
        servoGrSta = hardwareMap.get(Servo.class, "servoGrSta");
        servoTg1 = hardwareMap.servo.get("servo_tg_stanga");
        servoTg2 = hardwareMap.servo.get("servo_tg_dreapta");


        servoTg2.setDirection(Servo.Direction.REVERSE);
        servoGrSta.setDirection(Servo.Direction.REVERSE);
        servoGrDr.setDirection(Servo.Direction.REVERSE);
    }

    // refactorizare cod ca sa nu avem 100 de linii de cod

    private Action ghearaAction(double positionGrDr, double positionGrSta) {
        return packet -> {
            servoGrDr.setPosition(positionGrDr);
            servoGrSta.setPosition(positionGrSta);
            return false;
        };
    }

    private Action bratAction(double positionTg1, double positionTg2) {
        return packet -> {
            servoTg1.setPosition(positionTg1);
            servoTg2.setPosition(positionTg2);
            return false;
        };
    }

    public Action prindereGheara(){
        return ghearaAction(0.97, -0.89);
    }

    public Action lasareGheara(){
        return ghearaAction(0.78, 0.28);
    }

    public Action ridicareGhearaBrat(){
        return bratAction(0.52, 0.52);
    }

    public Action pozitieObligaGheara(){
        return bratAction(0.9, 0.9);
    }

    public Action prindeSampleBara(){
        return bratAction(0.81, 0.81);
    }

}