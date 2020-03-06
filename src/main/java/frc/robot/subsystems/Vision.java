
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
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

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
  private static CameraServer cam1;
  private static CameraServer cam2;

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
  int rumbleCounter = 0;

  //Servo visionServo;
  boolean servoToggle = false;
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Vision(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal()){
      ledDisable();
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      //visionServo = new Servo(Constants.VisionServoPort);
      cam1.getInstance().startAutomaticCapture(0);
      //cam2.getInstance().startAutomaticCapture(1);
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
      ledEnable();
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      SmartDashboard.putNumber("tx", x);
      SmartDashboard.putNumber("ty", y);
      SmartDashboard.putNumber("area", area);
    }
    else
    {
      //To Do: Develop Vision simulation
    }
    if(Math.abs(x) < Constants.VisionErrorAllowed)
    {  
      goRumble = 1;
    }
    else 
    {
      goRumble = 0;
    }
  } 
  
  public double estimateDistancePowerPort(){ //Note: CANNOT use this method for Loading Bay. The Limelight is most likely too close in height to the Loading Bay target to make this method effective.
    double distance = (Constants.PowerPortTargetHeight-Constants.LimelightMountHeight) / Math.tan(Constants.LimelightMountAngle + x);
    return distance;
  }

  static int RumbleState = 0;
  static int lastGoRumble = 0;
  static int goRumble = 0;

  public void rumbler(){
    if ((goRumble == 1) && (lastGoRumble == 0) && (RumbleState == 0))
    {
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
      Robot.operatorInterface.opJoystick.setRumble(RumbleType.kLeftRumble, 1);
      rumbleCounter = Constants.rumbleCount;
      RumbleState = 1;
    }

    if (RumbleState == 0)
    {
      SmartDashboard.putString("Rumble Status", "In indefinite hold");
    }//Rumbling done
    else if (RumbleState == 1) //Rumbling on first time
    {
      SmartDashboard.putString("Rumble Status", "RumbleRumble 1");
      if (rumbleCounter != 0)
        rumbleCounter --;
      else
      {
        Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
        Robot.operatorInterface.opJoystick.setRumble(RumbleType.kLeftRumble, 0);
        RumbleState = 2;//Wait some time with rumble off
        rumbleCounter = Constants.rumbleCountWait;
      }
    }
    else if (RumbleState == 2)//Rumbling off for first time
    { 
      SmartDashboard.putString("Rumble Status", "No RumbleRumble");
      if (rumbleCounter != 0)
        rumbleCounter --;
      else
      {
        Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
        Robot.operatorInterface.opJoystick.setRumble(RumbleType.kLeftRumble, 1);
        RumbleState = 3;//Rumbling on for second time
        rumbleCounter = Constants.rumbleCount;
      }
    }
    else if (RumbleState == 3)//Rumbling off for second time
    if (rumbleCounter != 0)
        rumbleCounter --;
    else
    {
      SmartDashboard.putString("Rumble Status", "RumbleRumble 2");
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
      Robot.operatorInterface.opJoystick.setRumble(RumbleType.kLeftRumble, 0);
      RumbleState = 0;//Rumbling done, wait for new start
    }

    lastGoRumble = goRumble;
/*
    if(rumbleCounter > 0 && rumbleCounter < 50 || rumbleCounter > 100 && rumbleCounter < 150){
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kRightRumble, 1);
    } if(rumbleCounter > 50 && rumbleCounter < 100 || rumbleCounter > 150){
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kRightRumble, 0);
    } if(rumbleCounter > 150) {
      rumbleOnce = true;
    }*/
  }

  /*public void servoTilt(double angle) {
    visionServo.setAngle(angle);
  }*/

  public static void ledDisable(){
    table.getEntry("ledMode").setNumber(1);
  }

  public static void ledEnable(){
    table.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new MonitorJeVois());
  }
}
