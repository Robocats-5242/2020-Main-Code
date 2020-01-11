/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  private static DoubleSolenoid solenoid;
  private static Compressor compressor;
  private String CommandName = "Pneumatics";
  private static boolean pneumaticState = false;


  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private void initPneumatics(){
//    pneumaticsSystem = new Pneumatics();//Creating here is circular !!!
    solenoid = new DoubleSolenoid(Constants.PneuStroke1Channel, Constants.PneuStroke2Channel);
    compressor = new Compressor();

    compressor.setClosedLoopControl(true);
  }

  public Pneumatics(){ 
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal() && Robot.useHardware())
      initPneumatics();
  }

  public static void setLiftPiston(boolean state){
    
    if(state == Constants.PneuLiftIn){
      pneumaticState = false;
      if (Robot.isReal() && Robot.useHardware())
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      pneumaticState = true;
      if (Robot.isReal() && Robot.useHardware())
        solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public boolean getPneumaticsState(){
    return pneumaticState;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
