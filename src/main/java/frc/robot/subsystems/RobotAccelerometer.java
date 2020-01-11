/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class RobotAccelerometer extends Subsystem {
  private static double averagingTracker[];
  //public static ADIS16470_IMU robotGyro;
  private static BuiltInAccelerometer imu;
  private static String CommandName = "Accelerometer";
  private static int averagingCount = 20;
  private static int averagingIndex = 0;
  private static double averagingSummation = 0.0;
  private static double averageValue = 0.0;
  private static double initialReading = 0.0;

  public RobotAccelerometer() {
    //imu = new ADIS16470_IMU();
    imu = new BuiltInAccelerometer(Accelerometer.Range.k2G);
    //robotGyro = new ADIS16470_IMU();
    //Initialize an array for our averaging filter
    averagingTracker = new double[averagingCount];
    double currentReading = getAccelZ();
    for (int x = 0; x < averagingCount; x++) 
    {
      averagingTracker[x] = currentReading;
    }
    averagingSummation = currentReading * averagingCount;
    averageValue = currentReading;
    averagingIndex = 0;
  }

  public static void updateAveraging(){
    double currentReading = getAccelZ();
    //Update the running summation by removing the oldest value and adding the newest
    averagingSummation = averagingSummation - averagingTracker[averagingIndex] + currentReading;
    //Update the oldest tracking value and replace with newest
    averagingTracker[averagingIndex] = currentReading;
    //Advance the tracking index ready for the next sample
    averagingIndex = averagingIndex + 1;
    if (averagingIndex == averagingCount)
      averagingIndex = 0;
    //Re-calculate the new average
    averageValue = averagingSummation / averagingCount;
  }

  public static double getAngleX(){
    //return robotGyro.getAccelZ();
    return 0;

  } 
  public static double getAngleY(){
    //return imu.getAngleY();
    return 0;
  } 
  public static double getAngleZ(){
    ///return imu.getAngleZ();
    return 0;
  } 

  public static double getAccelX(){
    //return imu.getAccelX();
    return imu.getX();
  } 

  public static double getAccelY(){
    //return imu.getAccelY();
    return imu.getY();
  } 

  public static double getAccelZ(){
    //return imu.getAccelZ();
    return imu.getZ();
  } 

  public static double getAccelZAverage(){
    return averageValue;
  }

  public static void captureOffset(){
    initialReading = getAccelZAverage();
    Robot.logMessage(CommandName, "Z Offset = " + initialReading);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
