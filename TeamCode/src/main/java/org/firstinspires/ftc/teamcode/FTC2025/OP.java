package org.firstinspires.ftc.teamcode.FTC2025;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Configurable
public class OP extends OpMode {
    static RobotBase robot;
    static double spinPower = 0.25;
    static double intakePower = 1;
    boolean intakeMode = false;
    boolean armMode = false;
    @Override
    public void init() {
        robot = new RobotBase(hardwareMap);
        robot.spinPosition(60);
        robot.spinPower(0);
        robot.armout();
    }

    @Override
    public void loop() {
        robot.ApriltagAim();

//        robot.ShooterAim();
        robot.move(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
        robot.shooter(gamepad1.right_trigger);
        //armMode
        if (gamepad1.left_bumper) armMode = !armMode;
        if(armMode){
            robot.armin();
            robot.riseball(1);
            robot.spinPower(1);
        }else{
            robot.armout();
            robot.riseball(0);
            robot.spinPower(0);
        }
        //intakeMode
        if(gamepad1.right_bumper) intakeMode = !intakeMode;
        if(intakeMode){
            robot.intake(intakePower);
            robot.spinPower(spinPower);
        }else{
            robot.intake(0);
            robot.spinPower(0);
        }
//        robot.spincolor();
        telemetry.addData("color1",robot.getColor1());
        telemetry.addData("color1",robot.getColor2());
        telemetry.addData("color1",robot.getColor3());
        telemetry.addData("tx",robot.limelight("tx"));
        telemetry.addData("ty",robot.limelight("ty"));
        telemetry.addData("id",robot.limelight("id"));
        telemetry.addData("Spin Position",robot.getSpinPosition());
        telemetry.addData("Aim Position",robot.getAimPosition());
    }
}
