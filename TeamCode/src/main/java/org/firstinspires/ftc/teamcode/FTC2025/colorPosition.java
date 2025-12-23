package org.firstinspires.ftc.teamcode.FTC2025;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class colorPosition extends OpMode {
    AnalogInput analog;
    CRServo servo;

    @Override
    public void init() {
        analog = hardwareMap.get(AnalogInput.class,"analog");
        servo = hardwareMap.get(CRServo.class,"spin");
        servo.setPower(0.5);
    }

    @Override
    public void loop() {
        double position = ((analog.getVoltage()/analog.getMaxVoltage())*300);
        servo.setPower(0);
        telemetry.addData("Position",position);
        telemetry.addData("Voltage",analog.getVoltage());
        telemetry.addData("MaxVoltage",analog.getMaxVoltage());
    }
}
