package org.firstinspires.ftc.teamcode.Mode.Servo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC2025.PIDFController;
import org.firstinspires.ftc.teamcode.FTC2025.RobotBase;
import org.firstinspires.ftc.teamcode.FTC2025.RobotHardware;

public class AimServo {
    ServoHardware servo;
    PIDFController AimPIDF;
    public AimServo(HardwareMap hardwareMap){servo = new ServoHardware(hardwareMap);}
    public double getAimPosition(){
        return (servo.AimAnalog.getVoltage() / servo.AimAnalog.getMaxVoltage()) * 360;
    }
    public void setAimPosition(double Target){

    }
}
