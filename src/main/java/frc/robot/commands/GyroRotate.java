package frc.robot.commands;

//import frc.robot.OI;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

public class GyroRotate extends Command{

    static double angle;
    static double currAngle = 0.0;
    static double angleDiff;
    static boolean doneTurn = false;
    static boolean turnRight;

    public GyroRotate(double angleYaw){
        requires(Robot.driveTrain);
        //requires(Robot.imu);
        angle = angleYaw;
    }

    @Override
    protected void initialize() {
        currAngle = Robot.imu.getAngleZ();
        Robot.driveTrain.setAutoFlag(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        currAngle = Robot.imu.getAngleZ();
        angleDiff = angle - currAngle;
        //Code similar to AlignWithTarget. Don't forget to add a P factor to control the speed of turn //CCW is positive
        Robot.driveTrain.setSpeedPercentAuto(-(angleDiff / angle) * Constants.AutoRotatekP, (angleDiff / angle) * Constants.AutoRotatekP);
        if(Math.abs(angleDiff) < Constants.AutoRotateError){
            doneTurn = true;
        }
    }

    @Override
    protected boolean isFinished() {
      return doneTurn;
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setSpeedPercentAuto(0, 0);
    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.driveTrain.setSpeedPercentAuto(0, 0);
    }

    public static void gyroRotate(double angleYaw){
        boolean isDone = false;
        double angle = angleYaw;
        double currAngle = Robot.imu.getAngleZ();
        double angleDiff = angle - currAngle;
        double currAngleDiff;
        double speed;
        double localMod = Constants.AutoRotatekP;
        while(!isDone){
            currAngle = Robot.imu.getAngleZ();
            currAngleDiff = angle - currAngle;
            if(Math.abs(currAngleDiff) >= 90)speed = Math.abs(currAngleDiff/angleDiff) * localMod + Constants.AutoRotateConstant;
            else if(Math.abs(currAngleDiff) >= 45)speed = Math.abs(currAngleDiff/angleDiff) * localMod * 0.5 + Constants.AutoRotateConstant;
            else speed = Math.abs(currAngleDiff/angleDiff) * localMod * 0.25 + Constants.AutoRotateConstant;
            if(currAngleDiff < -Constants.AutoRotateError){
                Robot.driveTrain.setSpeedPercentAuto(speed, -speed);
            }else if(currAngleDiff > Constants.AutoRotateError){
                Robot.driveTrain.setSpeedPercentAuto(-speed, speed);
            }else{
                isDone = true;
                Robot.driveTrain.setSpeedPercentAuto(0, 0);
            }
            Robot.driveTrain.updateDriveTrain();
        }
        Robot.driveTrain.fullStop();
    }
}