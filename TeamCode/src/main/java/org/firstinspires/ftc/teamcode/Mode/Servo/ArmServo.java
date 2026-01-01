package org.firstinspires.ftc.teamcode.Mode.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmServo {
    ServoHardware servo;
    public ArmServo(HardwareMap hardwareMap){
        servo = new ServoHardware(hardwareMap);
    }
}
