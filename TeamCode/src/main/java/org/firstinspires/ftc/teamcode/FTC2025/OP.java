package org.firstinspires.ftc.teamcode.FTC2025;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OP extends OpMode {
    RobotBase robot;
    @Override
    public void init() {
        robot.spinPosition(60);
    }

    @Override
    public void loop() {
        robot.ApriltagAim();
        robot.ShooterAim();
        robot.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        robot.shooter(gamepad1.right_trigger);
        telemetry.addData("color",robot.spincolor());
    }
}
