/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Constants;

//import com.ctre.phoenix.motorcontrol.ControlMode;

//import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
//import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DrivesWithJoysticks extends Command {
  private static String CommandName = "DrivesWithJoysticks";
  
  public DrivesWithJoysticks() {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //updateDriveFromJoystick();
  }

  public static void updateDriveFromJoystick(){
    double targetLeftSpeed;
    double targetRightSpeed;
    double currentLeftSpeed;
    double currentRightSpeed;
    double newLeftSpeed;
    double newRightSpeed;
  

    targetLeftSpeed =  Robot.operatorInterface.getControllerStickLeft();
    targetRightSpeed = Robot.operatorInterface.getControllerStickRight();

    if (Math.abs(targetLeftSpeed) < 0.10) {
      targetLeftSpeed = 0;
    }

    if (Math.abs(targetRightSpeed) < 0.10) {
      targetRightSpeed = 0;
    }

    currentLeftSpeed = Robot.driveTrain.getLeftSpeedPercent();
    currentRightSpeed = Robot.driveTrain.getRightSpeedPercent();

    if(Robot.operatorInterface.getControllerTriggerLeft() > 0.05){
      targetLeftSpeed*=.3;
      targetRightSpeed*=.3;
    }
    else if(Robot.operatorInterface.getControllerTriggerRight() > 0.05){
      targetLeftSpeed*=.5;
      targetRightSpeed*=.5;
    }
    
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonX)){
      targetLeftSpeed = Constants.CrawlSpeed;
      targetRightSpeed = Constants.CrawlSpeed;
    }

    newLeftSpeed = adjustSpeed(currentLeftSpeed, targetLeftSpeed, Constants.JoystickAccelleration, Constants.JoystickDecelleration);
    newRightSpeed = adjustSpeed(currentRightSpeed, targetRightSpeed, Constants.JoystickAccelleration, Constants.JoystickDecelleration);
    Robot.driveTrain.setSpeedPercentJoystick(newLeftSpeed, newRightSpeed);

    //Check if we need to adjust the lift height for hab climb
    if (Robot.operatorInterface.getPOV() == 0){
      Robot.liftSystem.moveLift(Constants.LiftManualSpeed);
    }
    else if (Robot.operatorInterface.getPOV() == 180){
      Robot.liftSystem.moveLift(-Constants.LiftManualSpeed);
    }

    //Check if we need to turn on the hab climb crawl motor
    if (Robot.operatorInterface.getPOV() == 90){
      Robot.habClimber.setCrawlerPower(Constants.HabCrawlSpeed);
    }
    else if (Robot.operatorInterface.getPOV() == 270){
      Robot.habClimber.setCrawlerPower(-Constants.HabCrawlSpeed);
    }
    else{
      Robot.habClimber.setCrawlerPower(0);
    }

    if (Robot.operatorInterface.getControllerButtonState(Constants.HabClimbButton) == false){
      //Not currently climbing so check if retracting
      //Not really the best way to do this but should work... kinda
      //Button to retract the rear climber back up
      if (Robot.operatorInterface.getControllerButtonState(Constants.HabRetractButton))
        Robot.habClimber.setClimberPower(Constants.ClimberRetractSpeed);
      else
        Robot.habClimber.setClimberPower(0);
    }
    
  }
  
  private static double adjustSpeed(double current, double target, double upDelta, double downDelta)
  {
    double result = 0.0;

    //If need to slow down...
    if (target < current){
      result = current - downDelta;
      //Check if gone too far
      if (result < target)
        result = target;
    }
    //Need to speed up
    else if (target > current){
      result = current + upDelta;
      //Check if gone too far
      if (result > target)
        result = target;
    }
    else
      result = target;
      
    return result;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

