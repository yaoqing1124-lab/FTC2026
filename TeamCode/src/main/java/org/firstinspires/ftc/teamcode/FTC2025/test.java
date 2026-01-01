package org.firstinspires.ftc.teamcode.FTC2025;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class test extends OpMode {
    CRServo servo;
    AnalogInput analog;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class,"servo");
        analog = hardwareMap.get(AnalogInput.class,"analog");
    }

    @Override
    public void loop() {

    }
}