package org.firstinspires.ftc.teamcode.FTC2025;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class t1221 extends OpMode {
    RobotBase robot;
    @Override
    public void init() {
        robot = new RobotBase(hardwareMap);
    }

    @Override
    public void loop() {
        robot.spinPosition(RobotBase.SpinPose1);
        int c1 = robot.isgreen();
        robot.spinPosition(RobotBase.SpinPose2);
        int c2 = robot.isgreen();
        robot.spinPosition(RobotBase.SpinPose3);
        int c3 = robot.isgreen();

        telemetry.addData("1",c1);
        telemetry.addData("2",c2);
        telemetry.addData("3",c3);
        telemetry.update();
    }
}
