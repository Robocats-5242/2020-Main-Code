
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.commands.*;

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

<<<<<<< HEAD
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  double txSim;
  double tySim;
  double taSim;

  public double x;
  public double y;
  double area;

=======
>>>>>>> cc7779ce477f86f64bfb07ccf46967ade0c467c9
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
    }
}

<<<<<<< HEAD
  public void updateVisionSimulation(){
  }

  public void updateVision(){
    if(Robot.isReal()){//read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    }else{
      //To Do: Develop Vision simulation
  }
  } 
=======
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
>>>>>>> cc7779ce477f86f64bfb07ccf46967ade0c467c9
  
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

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new MonitorJeVois());
  }

  public double getTargetError() {
    return xPosition;
  }
  
}
