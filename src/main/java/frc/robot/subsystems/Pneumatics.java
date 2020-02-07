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
  private static DoubleSolenoid solenoidClimb;
  private static Solenoid solenoidShift;
  private static Compressor compressor;
  private String CommandName = "Pneumatics";
  private static boolean pneumaticStateIntake = false;
  private static boolean pneumaticStateClimb = false;
  private static boolean pneumaticStateShifter = false;

  //Intake  - 1 double solenoid
  //Climber - 1 double solenoid
  //EVO Shifters - 1 standard solenoid

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private void initPneumatics(){
//    pneumaticsSystem = new Pneumatics();//Creating here is circular !!!
    solenoidIntake = new DoubleSolenoid(Constants.PneuStroke1Channel, Constants.PneuStroke2Channel);
    solenoidClimb = new DoubleSolenoid(Constants.PneuStroke3Channel, Constants.PneuStroke4Channel);
    solenoidShift = new Solenoid(Constants.PneuStroke5Channel);
    compressor = new Compressor();

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
        solenoidClimb.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      pneumaticStateClimb = true;
      if (Robot.isReal() && Robot.useHardware())
        solenoidClimb.set(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getPneumaticsStateClimb(){
    return pneumaticStateClimb;
  }

  public static void setShifterPiston(boolean state){
    
    if(state == Constants.PneuShiftLow){
      pneumaticStateShifter = false;
      if (Robot.isReal() && Robot.useHardware())
        solenoidShift.set(false);
    }
    else{
      pneumaticStateShifter = true;
      if (Robot.isReal() && Robot.useHardware())
        solenoidShift.set(true);
    }
  }

  public boolean getPneumaticsStateShifter(){
    return pneumaticStateShifter;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
