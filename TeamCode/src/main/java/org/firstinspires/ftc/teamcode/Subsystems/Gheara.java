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

    // asta ar trb sa fie pozitia stass de prins sample in orice program
    public Action prindereGheara(){
        return ghearaAction(0.94, -0.98);
    }

    // asta e pozitia stass de lasat orice sample in orice program
    public Action lasareGheara(){
        return ghearaAction(0.78, 0.1);
    }

    // asta e doar pentru autonomiile de la cos, pozitia pentru gheara de la
    // prindere sample 3
    //fosta pozitie pt caramida perete
    public Action deschideGhearaPentruSamplePerete(){
        return ghearaAction(0.7, 0.2);
    }

    // asta ar trebui sa fie stass pentru toate, brat gheara vertical
    public Action ridicareGhearaBrat(){
        return bratAction(0.52, 0.52);
    }

    // asta e pozitia cu care ar lua sample de pe teren(ground)
    //fosta pozitie obligaGheara 0.9, 0.9
    public Action pozitioneazaBratPentruSampleDePeTeren(){
        return bratAction(0.9, 0.9);
    }

    // asta e pozitia prin care bratul e pozitionat sa ia sample din observation zone
    public Action prindeSampleBara(){
        return bratAction(0.84, 0.84);
    }

}