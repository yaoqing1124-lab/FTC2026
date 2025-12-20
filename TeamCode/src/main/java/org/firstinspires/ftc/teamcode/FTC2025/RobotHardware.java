package org.firstinspires.ftc.teamcode.FTC2025;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotor lf, lb, rf, rb;
    public DcMotor shooter1, shooter2;
    public DcMotor intake;
    public CRServo aimX1, aimX2;
    public Servo aimY1, aimY2;
    public CRServo spin;
    public Servo arm;
    public Limelight3A limelight3A;
    public AnalogInput analog;

    public ColorSensor color1,color2;
    public void init(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        aimX1 = hardwareMap.get(CRServo.class, "aim1");
        aimX2 = hardwareMap.get(CRServo.class, "aim2");

        spin = hardwareMap.get(CRServo.class, "spin");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        analog = hardwareMap.get(AnalogInput.class,"analog");

        intake = hardwareMap.get(DcMotor.class,"intake");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
//       rf.setDirection(DcMotorSimple.Direction.REVERSE);
//       rb.setDirection(DcMotorSimple.Direction.REVERSE);

        color1 = hardwareMap.get(ColorSensor.class,"color1");
        color2 = hardwareMap.get(ColorSensor.class,"color2");

        arm = hardwareMap.get(Servo.class,"arm");
       }
}
