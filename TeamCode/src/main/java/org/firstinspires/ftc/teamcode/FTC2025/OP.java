package org.firstinspires.ftc.teamcode.FTC2025;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OP extends OpMode {
    RobotBase robot;
    @Override
    public void init() {
        robot = new RobotBase(hardwareMap);
//        robot.spinPosition(60);
        robot.spinPower(0);
        robot.armout();
    }

    @Override
    public void loop() {
//        robot.ApriltagAim();
//        robot.ShooterAim();
        robot.move(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        robot.shooter(gamepad1.right_trigger);
        if(gamepad1.right_bumper) robot.armin();
        if(gamepad1.left_bumper)robot.armout();
        robot.spinPower(0.3);
//        robot.spincolor();
        robot.riseball();
        robot.intake(1);
//        telemetry.addData("color1",robot.getColor1());
//        telemetry.addData("color1",robot.getColor2());
//        telemetry.addData("color1",robot.getColor3());
    }
}
