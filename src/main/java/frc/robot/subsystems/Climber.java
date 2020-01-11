/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private String CommandName = "Climber";
  VictorSPX climber;
  VictorSPX crawler;
  private double climberPower = 0;
  private double crawlerPower = 0;

  private void climberInit(){
    climber = new VictorSPX(Constants.CANClimberController);
    crawler = new VictorSPX(Constants.CANCrawlerController);

    climber.configFactoryDefault();
    crawler.configFactoryDefault();

    climber.configNominalOutputForward(0, Constants.ClimberkTimeoutMs);
    climber.configNominalOutputReverse(0, Constants.ClimberkTimeoutMs);
    climber.configPeakOutputForward(Constants.ClimberPIDpeakoutput, Constants.ClimberkTimeoutMs);
    climber.configPeakOutputReverse(-Constants.ClimberPIDpeakoutput, Constants.ClimberkTimeoutMs);
    climber.configOpenloopRamp(.3, Constants.ClimberkTimeoutMs);
    climber.setNeutralMode(NeutralMode.Brake);

    crawler.configNominalOutputForward(0, Constants.ClimberkTimeoutMs);
    crawler.configNominalOutputReverse(0, Constants.ClimberkTimeoutMs);
    crawler.configPeakOutputForward(Constants.ClimberPIDpeakoutput, Constants.ClimberkTimeoutMs);
    crawler.configPeakOutputReverse(-Constants.ClimberPIDpeakoutput, Constants.ClimberkTimeoutMs);
    crawler.configOpenloopRamp(.3, Constants.ClimberkTimeoutMs);
    crawler.setNeutralMode(NeutralMode.Brake);

  }

  public Climber(){
    //Initialize the drive train
    if (Robot.isReal() && Robot.useHardware()){
      climberInit();
    }
  }

  public void setClimberPower(double power){
    if (Robot.isReal() && Robot.useHardware()){
      climber.set(ControlMode.PercentOutput, power);
    }
    climberPower = power;
  }

  public double getClimberPower(){
    return climberPower;
  }

  public void setCrawlerPower(double power){
    if (Robot.isReal() && Robot.useHardware()){
      crawler.set(ControlMode.PercentOutput, power);
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
