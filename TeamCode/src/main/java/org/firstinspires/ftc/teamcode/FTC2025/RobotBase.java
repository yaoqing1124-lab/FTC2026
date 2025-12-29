package org.firstinspires.ftc.teamcode.FTC2025;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Configurable
public class RobotBase {
    RobotHardware robot;
    static PIDFController aimPIDF = new PIDFController(0.01, 0, 0.005, 0,5);
    static double SpinPose1 = 90;
    static double SpinPose2 = SpinPose1 + 100;
    static double SpinPose3 = SpinPose1 + 200;
    int spinPose = 1;
    public int color1;
    public int color2;
    public int color3;
    static double armInPosition=0.09;
    static double armOutPosition = 0.3;

    public RobotBase(HardwareMap hardwareMap) {
        robot = new RobotHardware(hardwareMap);
    }

    public void move(double lateral, double axial, double yaw) {
        double lfp = axial + lateral + yaw;
        double rfp = axial - lateral - yaw;
        double lbp = axial - lateral + yaw;
        double rbp = axial + lateral - yaw;

        robot.lf.setPower(lfp);
        robot.lb.setPower(lbp);
        robot.rf.setPower(rfp);
        robot.rb.setPower(rbp);
    }

    public void shooter(double shooterVelocity) {
        robot.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter1.setPower(shooterVelocity);
        robot.shooter2.setPower(-shooterVelocity);
    }

    public double limelight(String type) {
        robot.limelight3A.setPollRateHz(100);
        robot.limelight3A.start();
        robot.limelight3A.pipelineSwitch(0);
        LLResult result = robot.limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            switch (type) {
                case "tx":
                    return result.getTx();
                case "ty":
                    return result.getTy();
                case "ta":
                    return result.getTa();
                case "id":
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        return tag.getFiducialId();
                    }
            }
        }
        return 0;
    }

    public void ApriltagAim() {
        robot.aimX1.setPower(aimPIDF.data(-limelight("tx"), robot.aimX1.getPower()));
        robot.aimX2.setPower(aimPIDF.data(-limelight("tx"), robot.aimX2.getPower()));
    }
    public void setAimPower(double AimPower){
        robot.aimX1.setPower(AimPower);
        robot.aimX2.setPower(AimPower);
    }
    public void setAimPosition(double AimPosition){
        if (Math.abs(AimPosition - getAimPosition()) > 50) {
            if(getAimPosition()>150) {
                setAimPower(0.2);
            }else{
                setAimPower(-0.2);
            }
        } else {
            robot.aimX2.setPower(0);
        }
    }

    public void ShooterAim() {
        double ty = limelight("ty");
        robot.aimY1.setPosition(ty);
        robot.aimY2.setPosition(1 - ty);
    }

    public double color(String type) {
        switch (type) {
            case "R":
                return (robot.color1.red() + robot.color2.red()) / 2.0;
            case "G":
                return (robot.color1.green() + robot.color2.green()) / 2.0;
            case "B":
                return (robot.color1.blue() + robot.color2.blue()) / 2.0;
            default:
                return (robot.color1.alpha() + robot.color2.alpha()) / 2.0;
        }
    }

    public double getSpinPosition() {
        return (robot.SpinAnalog.getVoltage() / robot.SpinAnalog.getMaxVoltage()) * 300;
    }
    public double getAimPosition(){
        return (robot.AimAnalog.getVoltage() / robot.AimAnalog.getMaxVoltage()) * 300;
    }

    public void spinPosition(double Target) {
        if (Math.abs(Target - getSpinPosition()) > 10) {
            robot.spin.setPower(0.2);
        } else {
            robot.spin.setPower(0);
        }
    }
    public void spinPower(double Power){
        robot.spin.setPower(Power);
    }

    public int getcolor() {
        if (color("alpha") < 300) {
            return 0;   //no
        } else if (color("G") > color("B")) {
            return 1;   //green
        } else {
            return 2;   //purple
        }
    }
//    public int spincolor(){
//        switch (spinPose){
//            case 1:
//                spinPose = 2;
//                spinPosition(SpinPose2);
//                return isgreen();
//            case 2:
//                spinPose = 3;
//                spinPosition(SpinPose3);
//                return isgreen();
//            case 3:
//                spinPose = 4;
//                spinPosition(SpinPose1);
//                return isgreen();
//        }
//        return 0;
//    }
    public void spincolor(){
        spinPosition(SpinPose1);
        color1 = getcolor();
        spinPosition(SpinPose2);
        color2 = getcolor();
        spinPosition(SpinPose3);
        color3 = getcolor();
    }
    public void armin(){
        robot.arm.setPosition(armInPosition);
    }
    public void armout(){
        robot.arm.setPosition(armOutPosition);
    }

    public void riseball(double riseballPower){
        robot.riseball.setPower(riseballPower);
    }
    public void intake(double IntakePower){
        robot.intake.setPower(-IntakePower);
    }
    public double getArmPosition(){
        return robot.arm.getPosition();
    }
    public int getColor1() {return color1;}
    public int getColor2() {return color2;}
    public int getColor3() {return color3;}
}