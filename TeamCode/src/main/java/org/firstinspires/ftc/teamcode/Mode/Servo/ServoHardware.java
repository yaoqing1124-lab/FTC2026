package org.firstinspires.ftc.teamcode.Mode.Servo;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoHardware {
    public CRServo aimX1, aimX2;
    public Servo aimY1, aimY2;
    public CRServo spin;
    public Servo arm;
    public AnalogInput SpinAnalog;
    public AnalogInput AimAnalog;
    public ServoHardware(HardwareMap hardwareMap){
        aimX1 = hardwareMap.get(CRServo.class, "aimX1");
        aimX2 = hardwareMap.get(CRServo.class, "aimX2");
        aimY1 = hardwareMap.get(Servo.class, "aimY1");
        aimY2 = hardwareMap.get(Servo.class, "aimY2");
        aimY2.setDirection(Servo.Direction.REVERSE);

        spin = hardwareMap.get(CRServo.class, "spin");
        spin.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(Servo.class,"arm");

        AimAnalog = hardwareMap.get(AnalogInput.class,"AimAnalog");
        SpinAnalog = hardwareMap.get(AnalogInput.class,"SpinAnalog");
    }
}
