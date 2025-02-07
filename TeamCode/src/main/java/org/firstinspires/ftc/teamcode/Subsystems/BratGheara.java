package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BratGheara {
    private Servo servoTg1, servoTg2;

    public BratGheara(HardwareMap hardwareMap){
        servoTg1 = hardwareMap.servo.get("servo_tg_stanga");
        servoTg2 = hardwareMap.servo.get("servo_tg_dreapta");

        servoTg2.setDirection(Servo.Direction.REVERSE);
    }

    private Action bratAction(double positionTg1, double positionTg2) {
        return packet -> {
            servoTg1.setPosition(positionTg1);
            servoTg2.setPosition(positionTg2);
            return false;
        };
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
        return bratAction(0.81, 0.81);
    }
}
