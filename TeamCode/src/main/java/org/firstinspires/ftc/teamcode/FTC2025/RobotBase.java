package org.firstinspires.ftc.teamcode.FTC2025;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
public class RobotBase {
    RobotHardware robot;
    PIDFController ShooterPIDF = new PIDFController(0, 0, 0, 0);
    PIDFController aimPIDF = new PIDFController(0, 0, 0, 0);
    static final int SpinPose1 = 60;
    static final int SpinPose2 = 160;
    static final int SpinPose3 = 260;

    public void drive(double lateral, double axial, double yaw) {
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
        robot.shooter1.setPower(ShooterPIDF.data(shooterVelocity, robot.shooter1.getPower()));
        robot.shooter2.setPower(ShooterPIDF.data(-shooterVelocity, robot.shooter2.getPower()));
    }

    public double limelight(String type) {
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
//                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        double id = tag.getFiducialId();
                    return id;
                }
            }
        }
        return 0;
    }

    public void ApriltagAim(){
        robot.aimX1.setPower(aimPIDF.data(limelight("tx"),robot.aimX1.getPower()));
        robot.aimX2.setPower(aimPIDF.data(limelight("tx"),robot.aimX2.getPower()));
    }

    public void ShooterAim(){
        double ty = limelight("ty");
        robot.aimY1.setPosition(ty);
        robot.aimY2.setPosition(1-ty);
    }
    public double color(String type) {
        switch (type) {
            case "R":
                return (robot.color1.red() + robot.color2.red())/2.0;
            case "G":
                return (robot.color1.green()+robot.color2.green())/2.0;
            case"B":
                return (robot.color1.blue()+robot.color2.blue())/2.0;
            default:
                return (robot.color1.alpha()+robot.color2.alpha())/2.0;
        }
    }
    public void spinPosition(double Target) {
        double position = (robot.analog.getVoltage()/ robot.analog.getMaxVoltage())*300;
        if(Math.abs(Target - position)>10){
            robot.spin.setPower(0.3);
        }else{
            robot.spin.setPower(0);
        }
    }
    public int isgreen(){
        if(color("alpha")<300){
            return 0;   //no
        } else if (color("G")>color("B")) {
            return 1;   //green
        } else{
            return 2;   //purple
        }
    }
    public int[] spincolor(){
        spinPosition(SpinPose1);
        int c1 = isgreen();
        spinPosition(SpinPose2);
        int c2 = isgreen();
        spinPosition(SpinPose3);
        int c3 = isgreen();
        return new int[]{c1, c2, c3};
    }
    public void arm(){
        if(robot.arm.getPosition()>0.3){
            robot.arm.setPosition(0.07);
        }else{
            robot.arm.setPosition(0.5);
        }
    }
}