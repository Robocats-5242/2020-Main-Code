/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private String CommandName = "Climber";
  private VictorSP crawler;
  private double crawlerPower = 0;

  private void climberInit(){
    crawler = new VictorSP(Constants.PWMCrawler);
  }

  public Climber(){
    //Initialize the drive train
    if (Robot.isReal() && Robot.useHardware()){
      climberInit();
    }
  }

  public void setCrawlerPower(double power){
    if (Robot.isReal() && Robot.useHardware()){
      crawler.set(power);
    }
    crawlerPower = power;
  }

  public double getCrawlerPower(){
    return crawlerPower;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
