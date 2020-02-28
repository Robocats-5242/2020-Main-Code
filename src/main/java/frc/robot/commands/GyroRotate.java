package frc.robot.commands;
//import frc.robot.OI;
import edu.wpi.first.wpilibj.command.*;
import frc.robot.Robot;
import frc.robot.Constants;

public class GyroRotate extends Command{

    double angle;
    double currAngle = 0.0;
    double angleDiff;
    boolean doneTurn = false;

    public GyroRotate(double angleYaw){
        requires(Robot.driveTrain);
        //requires(Robot.imu);
        angle = angleYaw;
    }

    @Override
    protected void initialize() {
        //currAngle = Robot.imu.getAngleZ();
        Robot.driveTrain.setAutoFlag(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        //currAngle = Robot.imu.getAngleZ();
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
}