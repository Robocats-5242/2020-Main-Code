/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
import frc.robot.Robot;
//import com.ctre.phoenix.motorcontrol.ControlMode;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.subsystems.*;

public class LiftToHeight extends Command {
  double distanceTicks;
  // private double oldPosition;
  private double localOffset;
  private boolean localBlocking;
  private double localHeightInches;
  private boolean done = false;
  private static String CommandName = "LiftToHeight";


  public LiftToHeight(double heightInches, double offset, boolean blocking) {
    Robot.logMessage(CommandName, "constructor - " + heightInches + " - " + offset + " - " + blocking);
        // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.liftSystem);
    localBlocking = blocking;
    localHeightInches = heightInches;    
    localOffset = offset;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.logMessage(CommandName, "initialize" + " - " + localHeightInches + " - " + localOffset);
    //Trigger the lift to move
    Lift.liftToPositionInches(localHeightInches, localOffset);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Set the target height
//    Robot.logMessage(CommandName, "execute");
    //If blocking then wait until either reached the height, appear to be stalled or timed out
    if (localBlocking == true){
      done = !Lift.isBusy();
    }
    else
      done = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
//    Robot.logMessage(CommandName, "isFinished");
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
//    Robot.logMessage(CommandName, "end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
//    Robot.logMessage(CommandName, "interrupted");
    //If interrupted then hold the current position, wherever it is currently
//    Lift.liftToPositionInches(Lift.getLiftPositionInches(), 0);
  }

}