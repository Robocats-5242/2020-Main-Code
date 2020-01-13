/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class AlignWithTarget extends Command {
  //variables
  private static boolean isDone = true;
  
  public AlignWithTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    requires(Robot.visionSystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isDone = false;
    Robot.driveTrain.setAutoFlag(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
<<<<<<< HEAD
    if(Robot.visionSystem.x > 1){ //1 = 1 degree margin of error
      Robot.driveTrain.setSpeedPercentAuto(0, Math.abs(Robot.visionSystem.x) / -27 * Constants.HomingModifier);
    }else if(Robot.visionSystem.x < -1){
      Robot.driveTrain.setSpeedPercentAuto(Math.abs(Robot.visionSystem.x) / -27 * Constants.HomingModifier, 0);
    }else {
      isDone = true;
    }
    Robot.driveTrain.updateDriveTrain();
=======
    if(Math.abs(Robot.visionSystem.getTargetError()) > Constants.MaxAllowableTargetError) {
      Robot.driveTrain.setAutoFlag(true);
      
    }
>>>>>>> cc7779ce477f86f64bfb07ccf46967ade0c467c9
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.setSpeedPercentAuto(0, 0);
    Robot.driveTrain.updateDriveTrain();
    Robot.driveTrain.setAutoFlag(false);
  }
  

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.setSpeedPercentAuto(0, 0);
    Robot.driveTrain.updateDriveTrain();
    Robot.driveTrain.setAutoFlag(false);
  }
}
