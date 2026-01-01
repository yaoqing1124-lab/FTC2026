package org.firstinspires.ftc.teamcode.Mode.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FTC2025.PIDFController;

import java.lang.annotation.Target;

public class SpinServo {
    ServoHardware servo;
    PIDFController SpinPIDF = new PIDFController(0,0,0,0,0);
    static double MidSpinPositoin = 0;
    public SpinServo(HardwareMap hardwareMap){
        servo = new ServoHardware(hardwareMap);
    }
    public double getSpinPosition(){
        return (servo.SpinAnalog.getVoltage() / servo.SpinAnalog.getMaxVoltage()) * 360;
    }
    public void setSpinPositoin(double Target){
        servo.spin.setPower(SpinPIDF.data(getSpinPosition(),Target));
    }
}
