/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  private static DoubleSolenoid solenoidIntake;
  private static DoubleSolenoid solenoidShift;
  private static Solenoid solenoidClimb;
  private static Compressor compressor;
  private String CommandName = "Pneumatics";
  private static boolean pneumaticStateIntake = false;
  private static boolean pneumaticStateClimb = false;
  private static boolean pneumaticStateShifter = false;

  //Intake  - 1 double solenoid
  //Climber - 1 standard solenoid
  //EVO Shifters - 1 double solenoid

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private void initPneumatics(){
//    pneumaticsSystem = new Pneumatics();//Creating here is circular !!!
    solenoidIntake = new DoubleSolenoid(1, 0, 1);
    solenoidShift = new DoubleSolenoid(1, 2, 3);
    solenoidClimb = new Solenoid(1, 4);
    compressor = new Compressor(1);

    compressor.setClosedLoopControl(true);
  }

  public Pneumatics(){ 
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal() && Robot.useHardware())
      initPneumatics();
  }

  public static void setIntakePiston(boolean state){
    
    if(state == Constants.PneuIntakeIn){
      pneumaticStateIntake = false;
      if (Robot.isReal() && Robot.useHardware())
        solenoidIntake.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      pneumaticStateIntake = true;
      if (Robot.isReal() && Robot.useHardware())
        solenoidIntake.set(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getPneumaticsStateIntake(){
    return pneumaticStateIntake;
  }

  public static void setClimbPiston(boolean state){
    
    if(state == Constants.PneuClimbIn){
      pneumaticStateClimb = false;
      if (Robot.isReal() && Robot.useHardware())
        solenoidClimb.set(false);
    }
    else{
      pneumaticStateClimb = true;
      if (Robot.isReal() && Robot.useHardware())
        solenoidClimb.set(true);
    }
  }

  public boolean getPneumaticsStateClimb(){
    return pneumaticStateClimb;
  }

  public static void setShifterPiston(boolean state){
    
    if(state == Constants.PneuShiftLow){
      pneumaticStateShifter = false;
      if (Robot.isReal() && Robot.useHardware())
        solenoidShift.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      pneumaticStateShifter = true;
      if (Robot.isReal() && Robot.useHardware())
        solenoidShift.set(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getPneumaticsStateShifter(){
    return pneumaticStateShifter;
  }
 
  public void updatePneumatic(){
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonStickLeft)) setIntakePiston(!getPneumaticsStateIntake());
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonStickRight)) setShifterPiston(!getPneumaticsStateShifter());
    if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonHome)) setClimbPiston(!getPneumaticsStateClimb());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
