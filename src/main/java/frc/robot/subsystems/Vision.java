
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
import edu.wpi.cscore.MjpegServer;
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

  public Vision(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal()){
      try {
        Robot.logMessage(CommandName, "Creating JeVois SerialPort...");
        JeVoisVision = new SerialPort(115200,SerialPort.Port.kUSB);
        jeVoisAlive = true;
        JeVoisVision.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess );
        JeVoisVision.enableTermination();
        Robot.logMessage(CommandName, "JeVois GOOD");
      } catch (Exception e) {
        jeVoisAlive = false;
        Robot.logMessage(CommandName, "JeVois FAILED");
        e.printStackTrace();
      }  
  //    SerialPort JeVoisVision = new SerialPort(9600, SerialPort.Port.kOnboard);
  //    SerialPort JeVoisVision = new SerialPort(9600, SerialPort.Port.kUSB);
  //    SerialPort JeVoisVision = new SerialPort(9600, SerialPort.Port.kUSB1);
  //    SerialPort JeVoisVision = new SerialPort(9600, SerialPort.Port.kUSB2);
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

  Servo visionServo;
  boolean servoToggle = false;

  public Vision(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal()){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      table.getEntry("ledMode").setNumber(3);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      visionServo = new Servo(Constants.VisionServoPort);
    }else{
      txSim = 0;
      tySim = 0;
      taSim = 0;
    }
}

  public double getRetroTargetError(){
      //This function returns an error factor to be used to point in the direction of the retro-flective targets
      return xPosition;
  }

  public void updateVisionSimulation(){
  }

  private void triggerMeasurement(){
    if (Robot.isReal() && (jeVoisAlive == true) && (isBusy == false)){
      isBusy = true;
      JeVoisVision.writeString("GO\n");
      //Robot.logMessage(CommandName, "Vision triggered");
    }
  }

  public void updateVision(){
    if (isBusy == false)
      triggerMeasurement();
    scanBuffer();
  }

  private double stringToDouble(String inString){
    double temp;
    try {
      temp = Double.parseDouble(rXString);
      return temp;
    } catch (Exception e) {
      return 0;
    }  
    if(Robot.isReal()){//read values periodically
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      SmartDashboard.putNumber("tx", x);
      SmartDashboard.putNumber("ty", y);
      SmartDashboard.putNumber("area", area);

      if(Robot.operatorInterface.driveJoystick.getXButtonReleased()){
        servoToggle = !servoToggle;
      }
      if(!servoToggle) 
        servoTilt(180);
      else 
        servoTilt(140);
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
  
  }
  
  private void  scanBuffer() {
    String rXChar;
    if ((jeVoisAlive == true) && (isBusy == true)){
      //Check if there is a character in the buffer
      //Robot.logMessage(CommandName, "Waiting for JeVois");
      timeoutCounter = timeoutCounter + 1;
      if (timeoutCounter >= 50){//Sent message but didn't receive what we expected so reset and try again
        rXString = "";
        isBusy = false;
        timeoutCounter = 0;
      }

      while(JeVoisVision.getBytesReceived() > 0){
        //Character there so add to retrieved string and check if end of string marker
        rXChar = JeVoisVision.readString(1);
        //Robot.logMessage(CommandName, "Received '" + rXChar + "'");
        //Robot.logMessage(CommandName, "RX = " + rXChar);          
        //if (Objects.equals(rXChar, "X")){
        
        if (rXChar.charAt(0) == 'X'){
          //Characters received so far denote X position
          xPositionTemp = stringToDouble(rXString);// Double.parseDouble(rXString);
          rXString = "";
          //Robot.logMessage(CommandName, "Received X = " + xPositionTemp);
        }
        /*else if (Objects.equals(rXChar, "Y")){
          //Characters received so far denote Y position (not used currently)
          yPositionTemp = stringToDouble(rXString);//Double.parseDouble(rXString);
          rXString = "";
          //Robot.logMessage(CommandName, "Received Y = " +yPositionTemp);
        }
        else if (Objects.equals(rXChar, "T")){
          //Characters received so far denote timestamp (not used currently)
          rXString = "";
          //Robot.logMessage(CommandName, "Received T = ???");
        }
        */
        else if (rXChar.charAt(0) == 'G'){
          //End of sequence marker to indicate all results are good
          rXString = "";
          xPosition = xPositionTemp;
          yPosition = yPositionTemp;
          isBusy = false;
          //Robot.logMessage(CommandName, "Vision done" + Double.toString(xPosition));
        }
        else if (rXChar.charAt(0) == 'O'){}//Ignore "OK" and new lines
        else if (rXChar.charAt(0) == 'K'){}
        else if (Objects.equals(rXChar, "/n")){}
        else{
          rXString = rXString + rXChar;
        }
      }
    }
  }

  public boolean searchingIsFinished(){
    return !isBusy;
  }

  static int RumbleState = 0;
  static int lastGoRumble = 0;
  static int goRumble = 0;

  public void rumbler(){
    if ((goRumble == 1) && (lastGoRumble == 0) && (RumbleState == 0))
    {
      Robot.operatorInterface.driveJoystick.setRumble(RumbleType.kLeftRumble, 1);
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

  public void servoTilt(double angle) {
    visionServo.setAngle(angle);
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new MonitorJeVois());
  }

  public double getTargetError() {
    return xPosition;
  }
  
}
