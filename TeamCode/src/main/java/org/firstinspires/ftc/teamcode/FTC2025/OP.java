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
    }

    @Override
    public void loop() {
    }
}
