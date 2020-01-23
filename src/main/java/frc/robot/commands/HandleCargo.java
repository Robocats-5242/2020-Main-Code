/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class HandleCargo extends Command {
  private int localState;
  private boolean isDone = false; 
  private String CommandName = "HandleCargo";

  public HandleCargo(int state) {
    requires(Robot.intakeSystem);
    localState = state;
  }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
      Robot.logMessage(CommandName, "initialize");
      switch(localState)
      {
        case Constants.IntakeStateOff :
          Robot.intakeSystem.setIntakeSpeed(0);
          //Intake.setIntakeSpeed(0);
          break;
        case Constants.IntakeStateHold :
          Robot.intakeSystem.setIntakeSpeed(Constants.IntakeHoldSpeed);
          //Intake.setIntakeSpeed(Constants.IntakeHoldSpeed);
          break;
        case Constants.IntakeStateIn :
          Robot.intakeSystem.setIntakeSpeed(Constants.IntakeInSpeed);
          //Intake.setIntakeSpeed(Constants.IntakeInSpeed);
          break;
        case Constants.IntakeStateOut :
          Robot.intakeSystem.setIntakeSpeed(Constants.IntakeEjectSpeed);
          //Intake.setIntakeSpeed(Constants.IntakeEjectSpeed);
          break;
        default :
          Intake.setIntakeSpeed(0);
      }
      
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }
  
  /*
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.logMessage(CommandName, "initialize");
    //Set the intake motor direction
    if(localInOut == Constants.IntakeIn){
      Intake.setIntakeSpeed(Constants.IntakeInSpeed);
    }
    else {
      Intake.setIntakeSpeed(Constants.IntakeEjectSpeed);
    }    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Now check if we need to wait or not...
    //if((inOut == Constants.IntakeIn) && (!Intake.isBallIn())){//Asking for intake and ball is not yet in
    
      if (localInOut == Constants.IntakeIn){
        //Blocking and intaking so wait for the ball before finishing
        Robot.logMessage(CommandName, "Intaking");
        if (Intake.isBallIn() == true){
          Robot.logMessage(CommandName, "Ball in");
          //Yes, intaking and ball is captured
          isDone = true;
          //Keep the cargo held
          //Intake.setIntakeSpeed(Constants.IntakeHoldSpeed);
        }
      }
      else {//Ejecting the ball so make sure it is out
        Robot.logMessage(CommandName, "Ejecting");
        if (Intake.isBallIn() == false){
          Robot.logMessage(CommandName, "Ball out");
          //Yes, ejecting and ball is out
          //Need to delay a little though to allow the ball to really get out
          Timer.delay(Constants.CargoEjectDelay);
          isDone = true;
          //Disable the intake
          Intake.setIntakeSpeed(0);
        }
      }
    
   
  }
*/

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.logMessage(CommandName, "end");
    //if(localInOut == Constants.IntakeOut && !localInControl)
    //  Intake.setIntakeSpeed(0);
    //Intake.setIntakeSpeed(Constants.IntakeHoldSpeed);//Can't do this since we need the intake to remain running sometimes
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Intake.setIntakeSpeed(Constants.IntakeHoldSpeed);
  }
}
