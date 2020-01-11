/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.analog.adis16470.frc.ADIS16470_IMU;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class Accelerometer extends Subsystem {

  public static ADIS16470_IMU imu;
  private static String CommandName = "Accelerometer";

  public Accelerometer() {
    imu = new ADIS16470_IMU();
  }

  public static double getAngleX(){
    //Robot.logMessage(CommandName, "getting angle");
    return imu.getAngleX();
  } 
  public static double getAngleY(){
    return imu.getAngleY();
  } 
  public static double getAngleZ(){
    return imu.getAngleZ();
  } 

  public static double getAccelX(){
    return imu.getAccelX();
  } 
  public static double getAccelY(){
    return imu.getAccelY();
  } 
  public static double getAccelZ(){
    return imu.getAccelZ();
  } 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
