package org.firstinspires.ftc.teamcode.FTC2025;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Configurable
public class Blue1 extends LinearOpMode {
    RobotBase robot;
    static double spinPower = 0.23;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new RobotBase(hardwareMap);
        Follower follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);


        Pose startPose = new Pose(56, 8, Math.toRadians(180));
        Pose BlueGroup1 = new Pose(8,33,Math.toRadians(180));
        Pose BlueGroup2 = new Pose();
        Pose ShootFar = new Pose(50,20,Math.toRadians(180));
        follower.setPose(startPose);

        PathChain toBlueGroup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,new Pose(38, 40),new Pose(65,30),BlueGroup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(0.6)
                .setBrakingStrength(1)
                .build();
        PathChain toShootFar = follower.pathBuilder()
                .addPath(new BezierLine(BlueGroup1,ShootFar))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(0.3)
                .setBrakingStrength(1)
                .build();
        PathChain toBlueGroup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootFar,new Pose(50,64),new Pose(64,58),BlueGroup2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStart(0.6)
                .setBrakingStrength(1)
                .build();


        waitForStart();
//        robot.ApriltagAim();
        robot.intake(1);
        robot.spinPower(spinPower);
        follower.followPath(toBlueGroup1);
        while (opModeIsActive() && follower.isBusy())   follower.update();
        follower.followPath(toShootFar);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        follower.followPath(toShootFar);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        sleep(300);
    }
}