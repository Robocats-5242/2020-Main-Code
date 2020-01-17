
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  private static SerialPort JeVoisVision;
  private static String CommandName = "Vision";
  private static boolean jeVoisAlive = false;
  private static String rXString = "";
  private static double xPosition = 0; 
  private static double yPosition = 0;
  private static double xPositionTemp = 0; 
  private static double yPositionTemp = 0;
  private static boolean isBusy = false;
  private static int timeoutCounter = 0;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  double txSim;
  double tySim;
  double taSim;

  public double x;
  public double y;
  double area;

  boolean rumbleOnce = false;

  public Vision(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal()){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      table.getEntry("ledMode").setNumber(3);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
    }else{
      txSim = 0;
      tySim = 0;
      taSim = 0;
    }
}

  public void updateVisionSimulation(){
  }

  public void updateVision(){
    if(Robot.isReal()){//read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("area", area);
    }else{
      //To Do: Develop Vision simulation
  }
    if(Math.abs(x) < 2 && !rumbleOnce){
      rumbler();
      rumbleOnce = true;
    } else if (Math.abs(x) > 2 && rumbleOnce) rumbleOnce = false;
  } 
  
  public double estimateDistancePowerPort(){ //Note: CANNOT use this method for Loading Bay. The Limelight is most likely too close in height to the Loading Bay target to make this method effective.
    double distance = (Constants.PowerPortTargetHeight-Constants.LimelightMountHeight) / Math.tan(Constants.LimelightMountAngle + x);
    return distance;
  }

  public void rumbler(){
    OI.driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
    OI.driveJoystick.setRumble(RumbleType.kRightRumble, 1);
    try{
        Thread.sleep(100);
    }catch (InterruptedException e){
    }
    OI.driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
    OI.driveJoystick.setRumble(RumbleType.kRightRumble, 0);
    try{
        Thread.sleep(100);
    }catch (InterruptedException e){
    }
    OI.driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
    OI.driveJoystick.setRumble(RumbleType.kRightRumble, 1);
    try{
        Thread.sleep(100);
    }catch (InterruptedException e){
    }
    OI.driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
    OI.driveJoystick.setRumble(RumbleType.kRightRumble, 0);
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new MonitorJeVois());
  }
}
