/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FindLine extends Command {
  private String CommandName = "FindLine";

  public FindLine() {
    Robot.logMessage(CommandName, "constructor");
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.logMessage("----", "----");
    Robot.logMessage(CommandName, "initialize");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.logMessage(CommandName, "execute");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Robot.logMessage(CommandName, "isFinished");
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.logMessage(CommandName, "end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.logMessage(CommandName, "interrupted");
  }
}
