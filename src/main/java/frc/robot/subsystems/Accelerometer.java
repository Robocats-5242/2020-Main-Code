/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import frc.robot.Constants;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class Accelerometer extends Subsystem {

  public static PigeonIMU imu;
  private static String CommandName = "Accelerometer";

  public Accelerometer() {
    imu = new PigeonIMU(Constants.CANPigeon);
  }

  public static boolean isPigeonReady(){
    if(imu.getState() == PigeonState.Ready) return true;
    else return false;
  }

  public static double getAngleX(){
    //Robot.logMessage(CommandName, "getting angle");
    double[] accel = new double[3];
    imu.getAccelerometerAngles(accel);
    return accel[0];
  } 
  public static double getAngleY(){
    double[] accel = new double[3];
    imu.getAccelerometerAngles(accel);
    return accel[1];
  } 
  public static double getAngleZ(){
    double[] accel = new double[3];
    imu.getAccelerometerAngles(accel);
    return accel[2];
  } 

  public static short getAccelX(){
    short[] accel = new short[3];
    imu.getBiasedAccelerometer(accel);
    return accel[0];
  } 
  public static short getAccelY(){
    short[] accel = new short[3];
    imu.getBiasedAccelerometer(accel);
    return accel[1];
  } 
  public static short getAccelZ(){
    short[] accel = new short[3];
    imu.getBiasedAccelerometer(accel);
    return accel[2];
  } 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
