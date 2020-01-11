/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.Robot;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  static TalonSRX intake;
  static DigitalInput limitSwitch;
  private static String CommandName = "Intake";
  private static boolean limitSwitchSimulation = false;
  private static int limitSwitchCounterSimulation = 0;
  private static double targetIntakeSpeed = 0;
  private static double intakeSpeed = 0;
  public Intake(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal() && Robot.useHardware()){
      intake = new TalonSRX(Constants.CANIntakeController);
      limitSwitch = new DigitalInput(0);
    }
  }

  public static boolean isBallIn(){
    if (Robot.isReal() && Robot.useHardware())
      return !limitSwitch.get();//ToDo : Check polarity. True SHOULD mean the cargo is in
    else
      return limitSwitchSimulation;
  }

  public static void setIntakeSpeed(double speed){
    Robot.logMessage(CommandName, "Target intake Speed = " + speed);
    targetIntakeSpeed = speed;
    if (Robot.isSimulation() == true){
      if ((speed > 0) && (limitSwitchSimulation == false)){
        limitSwitchCounterSimulation = -100;//Wait some simulation time before updating limit switch
        Robot.logMessage(CommandName, "Intaking ball");
      }
      else if((speed < 0) && (limitSwitchSimulation == true)){
        limitSwitchCounterSimulation = 100;
        Robot.logMessage(CommandName, "Ejecting ball");
      }
    }
  }

  public static double getIntakeSpeed(){
    return intakeSpeed;
  }

  public void updateIntakeSimulation(){
    if (limitSwitchCounterSimulation < 0){
      limitSwitchCounterSimulation ++;
      if (limitSwitchCounterSimulation == 0)
        limitSwitchSimulation = true;
//      Robot.logMessage(CommandName, "updateIntakeSimulation : " + limitSwitchCounterSimulation);
    }
    else if (limitSwitchCounterSimulation > 0){
      limitSwitchCounterSimulation --;
      if (limitSwitchCounterSimulation == 0)
        limitSwitchSimulation = false;
//      Robot.logMessage(CommandName, "updateIntakeSimulation : " + limitSwitchCounterSimulation);
    }
  }

  public void updateIntake(){
    intakeSpeed = targetIntakeSpeed;
    if ((targetIntakeSpeed > 0) && (isBallIn() == true))//Intaking the ball but already captured so make sure only holding
      intakeSpeed = Constants.IntakeHoldSpeed;

//    Robot.logMessage(CommandName, "intakeSpeed = " + intakeSpeed);
    if (Robot.isReal() && Robot.useHardware())
      intake.set(ControlMode.PercentOutput, intakeSpeed);

  }


  @Override
  public void initDefaultCommand() {
  }
}
