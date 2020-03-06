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
        doneTurn = false;
        double speed = 0;
        angle = angleYaw;
        currAngle = Robot.imu.getAngleZ();
        Robot.driveTrain.setAutoFlag(true);
        if(angle == 0) angle = 0.001;
        if(angle > currAngle) turnRight = false;
        else turnRight = true;
        while(!doneTurn){
            currAngle = Robot.imu.getAngleZ();
            if(turnRight) angleDiff = currAngle - angle;
            else angleDiff = angle - currAngle;
            speed = Math.abs(Math.min((angleDiff / angle), 1) * Constants.AutoRotatekP);
            SmartDashboard.putNumber("Turn Speed", speed);
            //Code similar to AlignWithTarget. Don't forget to add a P factor to control the speed of turn //CCW is positive 
            if(turnRight) Robot.driveTrain.setSpeedPercentAuto(speed + Constants.AutoRotateConstant, -speed - Constants.AutoRotateConstant);
            else Robot.driveTrain.setSpeedPercentAuto(-speed - Constants.AutoRotateConstant, speed + Constants.AutoRotateConstant);
            if(Math.abs(angleDiff) < Constants.AutoRotateError){
                doneTurn = true;
            }
            Robot.driveTrain.updateDriveTrain();
            }
            Robot.driveTrain.setSpeedPercentAuto(0, 0);
            Robot.driveTrain.updateDriveTrain();
            try{
                Thread.sleep(500);
            } catch(InterruptedException ex){

            }
        }
}