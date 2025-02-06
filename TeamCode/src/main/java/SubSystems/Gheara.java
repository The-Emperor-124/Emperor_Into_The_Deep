package SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AlbastruCos;
import org.firstinspires.ftc.teamcode.AutoBaraRosu;

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

    public class PrindereGheara implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servoGrDr.setPosition(0.97);
            servoGrSta.setPosition(-0.89);
            return false;
        }
    }

    public Action prindereGheara() {
        return new PrindereGheara();
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