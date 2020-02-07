/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.VictorSP;

import frc.robot.Robot;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  static VictorSP intake;
  private static String CommandName = "Intake";
  private static double targetIntakeSpeed = 0;
  private static double intakeSpeed = 0;
  public Intake(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal() && Robot.useHardware()){
      intake = new VictorSP(1);
      intake.setInverted(true);
    }
  }

  public static void setIntakeSpeed(double speed){
    //Robot.logMessage(CommandName, "Target intake Speed = " + speed);
    targetIntakeSpeed = speed;
    }

  public static double getIntakeSpeed(){
    return intakeSpeed;
  }

  public void updateIntakeSimulation(){
    
  }

  public void updateIntake(){
    detectIntakeSpeed();
    intakeSpeed = targetIntakeSpeed;
    intake.set(intakeSpeed);

  }

  public void detectIntakeSpeed(){
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonA)) setIntakeSpeed(Constants.IntakeHoldSpeed);
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonB)) setIntakeSpeed(0);
  }


  @Override
  public void initDefaultCommand() {
  }
}
