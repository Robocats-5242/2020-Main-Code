/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class BalanceRobot extends Command {
  private String CommandName = "BalanceRobot";
  private double climberPower = 0;

  public BalanceRobot() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //When climb button pressed start the drive train creeping 
    Robot.driveTrain.setSpeedPercentAuto(Constants.AutoHabCreepSpeed, Constants.AutoHabCreepSpeed);
    Robot.driveTrain.setAutoFlag(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Measure the robot inclination
    double zAccelleration = Robot.imu.getAccelZAverage();
    double climberError = zAccelleration - Constants.ClimberTargetZ;
    climberPower = climberError * 5;//ToDo : Check polarity
    Robot.habClimber.setClimberPower(climberPower);
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
    //Disable the creeping and hab climber
    Robot.driveTrain.setAutoFlag(false);
    Robot.habClimber.setClimberPower(0);
  }
}

