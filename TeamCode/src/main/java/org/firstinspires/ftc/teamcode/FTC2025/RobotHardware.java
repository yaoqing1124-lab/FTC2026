package org.firstinspires.ftc.teamcode.FTC2025;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware{
    public DcMotor lf, lb, rf, rb;
    public DcMotor shooter1, shooter2;
    public DcMotor intake;
    public DcMotor riseball;
    public CRServo aimX1, aimX2;
    public Servo aimY1, aimY2;
    public CRServo spin;
    public Servo arm;
    public Limelight3A limelight3A;
    public AnalogInput analog;
    public ColorSensor color1,color2;

    public RobotHardware(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        shooter1 = hardwareMap.get(DcMotor.class, "shooterU");
        shooter2 = hardwareMap.get(DcMotor.class, "shooterD");

        aimX1 = hardwareMap.get(CRServo.class, "TURF");
        aimX2 = hardwareMap.get(CRServo.class, "TURB");
        aimY1 = hardwareMap.get(Servo.class, "LS");
        aimY2 = hardwareMap.get(Servo.class, "RS");

        spin = hardwareMap.get(CRServo.class, "SOR");
        spin.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        analog = hardwareMap.get(AnalogInput.class,"analog");

        intake = hardwareMap.get(DcMotor.class,"Intake");

//        lf.setDirection(DcMotorSimple.Direction.REVERSE);
//        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        riseball = hardwareMap.get(DcMotor.class,"LFTM");

        color1 = hardwareMap.get(ColorSensor.class,"CS1");
        color2 = hardwareMap.get(ColorSensor.class,"CS2");

        arm = hardwareMap.get(Servo.class,"LFTS");
       }
}
