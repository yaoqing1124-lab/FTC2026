package org.firstinspires.ftc.teamcode.T;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class gamepad extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.addData("R tigger",gamepad1.right_trigger);
        telemetry.addData("L tigger",gamepad1.left_trigger);
        telemetry.addData("dpad U",gamepad1.dpad_up);
        telemetry.addData("dpad R",gamepad1.dpad_right);
        telemetry.addData("dpad L", gamepad1.dpad_left);
        telemetry.addData("dpad D",gamepad1.dpad_down);
        telemetry.addData("R stick y",gamepad1.right_stick_y);
        telemetry.addData("R stick X",gamepad1.right_stick_x);
        telemetry.addData("o",gamepad1.circle);
    }
}
