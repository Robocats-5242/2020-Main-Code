/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveToPosition extends Command {
  private static double localDistanceInches;
  private static double localSpeed;
  private static double localStopDistance;
  private static int direction;
  private static boolean driveStopped = false;
  private String CommandName = "DriveToPosition";

  public DriveToPosition(double distanceInches, double speed, double stopDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    localDistanceInches = distanceInches;
    localSpeed = speed;
    localStopDistance = stopDistance;
    if (localDistanceInches < 0)
      direction = -1;
    else
      direction = 1;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
//      Robot.driveTrain.driveDistanceStraight(localDistanceInches, localSpeed, localStopDistance);
    Robot.logMessage(CommandName, "initialize");
    Robot.driveTrain.setAutoFlag(true);
    Robot.driveTrain.resetEncoders();//Reset the encoders so we can simply count from here
    //ToDo : Need to come up with a reasonable ultrasonic simulation methodology
    if (Robot.isSimulation())
      Robot.driveTrain.setUltrasonicValueSimulation(30);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Drive to Pos Running?", "Yes");
    double distanceTraveled;
    double leftDistanceTraveled;
    double rightDistanceTraveled;
    //double leftRightDistanceDelta;
    //double leftRightSpeedCorrection;

    leftDistanceTraveled  = Robot.driveTrain.getLeftEncoderInches();
    rightDistanceTraveled = Robot.driveTrain.getRightEncoderInches(); 
    distanceTraveled = (leftDistanceTraveled + rightDistanceTraveled) / 2;//Average the left and right encoders
    if (Math.abs(distanceTraveled) >= Math.abs(localDistanceInches)){//Check if gone the entire distance (note, direction is important)
      driveStopped = true;
    }
    /*else if ((Robot.driveTrain.getUltrasonicRange() < localStopDistance) && (localDistanceInches > 0.0)){//Check if too close. Can't use for reverse, but don't need at the moment
      driveStopped = true;
    }*/
    else{//Otherwise make sure driving straight
      //leftRightDistanceDelta = leftDistanceTraveled - rightDistanceTraveled;
      //leftRightSpeedCorrection = leftRightDistanceDelta * Constants.DriveStraightPGain;
      Robot.driveTrain.setSpeedPercentAuto(direction * (localSpeed/* - leftRightSpeedCorrection*/), direction * (localSpeed/* + leftRightSpeedCorrection*/));
    }
    Robot.driveTrain.updateDriveTrain();
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return driveStopped;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.logMessage(CommandName, "end");
    SmartDashboard.putString("Drive to Pos Running?", "No more");
    Robot.driveTrain.setSpeedPercentAuto(0, 0);
    Robot.driveTrain.updateDriveTrain();
    Robot.driveTrain.setAutoFlag(false);//Return control back to the joystick
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.logMessage(CommandName, "interrupted");
    Robot.driveTrain.setSpeedPercentAuto(0, 0);
    Robot.driveTrain.updateDriveTrain();
    Robot.driveTrain.setAutoFlag(false);//Return control back to the joystick
  }

  public static void driveToPosition(double distanceInches, double speed, double stopDistance){
    Robot.driveTrain.resetEncoders();
    driveStopped = false;
    double localDistanceInches = distanceInches;
    double localSpeed = speed;
    double localStopDistance = stopDistance;
    if (localDistanceInches < 0)
      direction = -1;
    else
      direction = 1;
    SmartDashboard.putString("Drive to Pos Running?", "Yes");
      double distanceTraveled;
      double leftDistanceTraveled;
      double rightDistanceTraveled;
      //double leftRightDistanceDelta;
      //double leftRightSpeedCorrection;
      while(!driveStopped){
        leftDistanceTraveled  = Robot.driveTrain.getLeftEncoderInches();
        rightDistanceTraveled = Robot.driveTrain.getRightEncoderInches(); 
        distanceTraveled = (leftDistanceTraveled + rightDistanceTraveled) / 2;//Average the left and right encoders
        if (Math.abs(distanceTraveled) >= Math.abs(localDistanceInches)){//Check if gone the entire distance (note, direction is important)
          driveStopped = true;
        }
        else{//Otherwise make sure driving straight
          //leftRightDistanceDelta = leftDistanceTraveled - rightDistanceTraveled;
          //leftRightSpeedCorrection = leftRightDistanceDelta * Constants.DriveStraightPGain;
          Robot.driveTrain.setSpeedPercentAuto(direction * (localSpeed), direction * (localSpeed));
        }
        Robot.driveTrain.updateDriveTrain();
      }
      Robot.driveTrain.setSpeedPercentAuto(0, 0);
      Robot.driveTrain.updateDriveTrain();
    }
} 
