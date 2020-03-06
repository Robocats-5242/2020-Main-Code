/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    if(Robot.visionSystem.x > Constants.VisionErrorAllowed){ 
      Robot.driveTrain.setSpeedPercentAuto((Math.abs(Robot.visionSystem.x) / 27) * Constants.HomingModifier, (Math.abs(Robot.visionSystem.x) / -27) * Constants.HomingModifier);
    }else if(Robot.visionSystem.x < -Constants.VisionErrorAllowed){
      Robot.driveTrain.setSpeedPercentAuto((Math.abs(Robot.visionSystem.x) / -27) * Constants.HomingModifier, (Math.abs(Robot.visionSystem.x) / 27) * Constants.HomingModifier);
    }else {
      Robot.driveTrain.setSpeedPercentAuto(0, 0);
      isDone = true;
    }
    Robot.driveTrain.updateDriveTrain();
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
    Robot.driveTrain.setAutoFlag(false);
  }
  

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.setSpeedPercentAuto(0, 0);
    Robot.driveTrain.setAutoFlag(false);
  }

  public static void alignWithTarget(){
    boolean isDone = false;
    Robot.visionSystem.updateVision();
    try{
      Thread.sleep(500);
    } catch(InterruptedException ex){
      
    }
    while(!isDone){
      SmartDashboard.putString("Auto is...", "Homing");
      Robot.visionSystem.updateVision();
      if(Robot.visionSystem.x > Constants.VisionErrorAllowed){ 
        Robot.driveTrain.setSpeedPercentAuto((Math.abs(Robot.visionSystem.x) / 27) * Constants.HomingModifier, (Math.abs(Robot.visionSystem.x) / -27) * Constants.HomingModifier);
      }else if(Robot.visionSystem.x < -Constants.VisionErrorAllowed){
        Robot.driveTrain.setSpeedPercentAuto((Math.abs(Robot.visionSystem.x) / -27) * Constants.HomingModifier, (Math.abs(Robot.visionSystem.x) / 27) * Constants.HomingModifier);
      }else {
        Robot.driveTrain.setSpeedPercentAuto(0, 0);
        isDone = true;
      }
      Robot.driveTrain.updateDriveTrain();
    }
    SmartDashboard.putString("Auto is...", "Doing nothing");
  }

  public static void alignWithTargetTeleOp(){
    Robot.driveTrain.setAutoFlag(true);
    alignWithTarget();
    Robot.driveTrain.setAutoFlag(false);
  }

  public static void updateAutoFocus(){
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonY)) alignWithTargetTeleOp();
  }
}
