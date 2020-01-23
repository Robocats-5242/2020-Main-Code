/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.*;
import frc.robot.subsystems.*;

public class HandleHatch extends Command {
  private boolean localUpDown;
  private boolean localBlock;
  private boolean isDone = false; 
  
  public HandleHatch(boolean upDown, boolean block) {
    requires(Robot.intakeSystem);
    this.localUpDown = upDown;
    this.localBlock = block;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double offset;
    double currentPosition;

    //Which state are we looking for?
    if (localUpDown == Constants.HatchDownState)
      offset = 0.0;
    else
      offset = Constants.HatchDepositDelta;

    //Get the current target position
    //Changed from Using the current position rather than the current target position allows setting based on mid flight position if necessary
    //Might need to change to target position rather than current position if behavior feels wrong
    currentPosition = Lift.getLiftPositionTarget();
    //Move list to correct position
    Lift.liftToPositionInches(currentPosition, offset);

    //Now check if we need to wait for move completion or not...
    if (localBlock == true)
      isDone = (Lift.isBusy());
    else
      isDone = true;//Not blocking so finished
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.setIntakeSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
