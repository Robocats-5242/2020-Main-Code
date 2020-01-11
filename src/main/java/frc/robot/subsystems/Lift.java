/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  static TalonSRX liftMotor;
  static VictorSPX liftFollower;
  static double liftPositionTarget = 0;
  static double liftPositionTargetTotal = 0;
  private static String CommandName = "Lift";
  private static int encoderTargetCountSimulation = 0;//Set to random value to check that we actually reset correctly
  private static int encoderCountSimulation = -42667;//Set to random value to check that we actually reset correctly
  private static double liftVelocitySimulation = 0.0;//ToDo : Need to write simulation code for this

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static void liftToPositionTicks(int heightTicks) {
    Robot.logMessage(CommandName, "liftToPositionTicks");

    if (Robot.isReal() && Robot.useHardware())
      liftMotor.set(ControlMode.Position, heightTicks);
    else{
      encoderTargetCountSimulation = heightTicks;
      updateLiftSimulation();
    }
  }

  public static void liftToPositionInches(double heightInches, double offset) {
    //Note where we were asked to go to, excluding the offset
    liftPositionTarget = heightInches;

    //The actual target is the selected height plus the hatch offset
    double target = liftPositionTarget + offset;
    liftPositionTargetTotal = target;

    //Make sure not too high/low
    if (target > Constants.LiftMaxHeight)
      target = Constants.LiftMaxHeight;
    else if (target < 0)
      target = 0;

    //System.out.println("Lift height = " + target);
    Robot.logMessage(CommandName, "Lift target height = " + target + " - " + target * Constants.LiftTicksPerInch);
    liftToPositionTicks((int)(target * Constants.LiftTicksPerInch));
  }

  private void configureMotors(){
    liftMotor    = new TalonSRX(Constants.CANLiftMasterController);
    liftFollower = new VictorSPX(Constants.CANLiftFollowerController);

    
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.LiftkkPIDLoopIdx, Constants.LiftkTimeoutMs);
    liftMotor.setSensorPhase(Constants.LiftkSensorPhase);
    liftMotor.setInverted(Constants.LiftkMotorInvert);
    liftMotor.configNominalOutputForward(0, Constants.LiftkTimeoutMs);
    liftMotor.configNominalOutputReverse(0, Constants.LiftkTimeoutMs);
    liftMotor.configPeakOutputForward(Constants.LiftPIDpeakoutputUp, Constants.LiftkTimeoutMs);
    liftMotor.configPeakOutputReverse(-Constants.LiftPIDpeakoutputDown, Constants.LiftkTimeoutMs);
    liftMotor.configAllowableClosedloopError(Constants.LiftkkPIDLoopIdx, Constants.LiftPIDmaxerror);
    liftMotor.config_kP(Constants.LiftkkPIDLoopIdx, Constants.LiftPIDkP, Constants.LiftkTimeoutMs);
    liftMotor.config_kI(Constants.LiftkkPIDLoopIdx, Constants.LiftPIDkI, Constants.LiftkTimeoutMs);
    liftMotor.config_kD(Constants.LiftkkPIDLoopIdx, Constants.LiftPIDkD, Constants.LiftkTimeoutMs);
    liftMotor.config_kF(Constants.LiftkkPIDLoopIdx, Constants.LiftPIDkF, Constants.LiftkTimeoutMs);
    /* Set the quadrature (relative) sensor to match absolute */
    liftMotor.setSelectedSensorPosition(0, Constants.LiftkkPIDLoopIdx, Constants.LiftkTimeoutMs);
    //liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 10, Constants.LiftkTimeoutMs);
    //Configure lift follower
    liftFollower.follow(liftMotor);

  }

  //Constructor creates the motor controller connections and configures them
  public Lift(){
    Robot.logMessage(CommandName, "constructor");
    if (Robot.isReal() && Robot.useHardware())
      configureMotors();
    resetEncoder();
  }

  private  void resetEncoder(){
    //ToDo : Check the parameters. They are supposed to be the count, PID & timeout values not the CAN ID
//    lift.setSelectedSensorPosition(11,1,1);
    if (Robot.isReal() && Robot.useHardware())
      liftMotor.setSelectedSensorPosition(0,0,Constants.LiftkTimeoutMs);
    else
      encoderCountSimulation = 0;
  }

  public static double getLiftPositionTicks(){
    if (Robot.isReal() && Robot.useHardware())
      return liftMotor.getSelectedSensorPosition();
    else
      return encoderCountSimulation;
  }

  public static double getLiftPositionInches(){
      return getLiftPositionTicks() / Constants.LiftTicksPerInch;
  }

  public void moveLift(double moveDelta){
    //Move to where we are targetting right now plus some delta
    //Offset = 0 since using the total target as the starting point
    liftToPositionInches(liftPositionTargetTotal + moveDelta, 0);
  }

  public static double getLiftPositionTarget(){
    return liftPositionTarget;
  }

  public static double getLiftPositionError(){
    return getLiftPositionInches() - getLiftPositionTarget();
  }

  public static double getLiftPositionErrorTotal(){
    return getLiftPositionInches() - liftPositionTargetTotal;
  }

  public static double getLiftPositionErrorPID(){
    if (Robot.isReal() && Robot.useHardware())
      return liftMotor.getClosedLoopError();
    else
      return 0;
  }

    public static double getLiftVelocity(){
    if (Robot.isReal() && Robot.useHardware())
      return liftMotor.getSelectedSensorVelocity();
    else
      return liftVelocitySimulation;
  }

  public static boolean isBusy(){
    //Use various measurements to determine if the lift is busy moving to where we asked it to
    //Options are...
    //Check the velocity
    //Check the motor output power
    //Check the absolute position error
    //Check the PID calculated position error

//    Robot.logMessage(CommandName, "isBusy - " + getLiftVelocity() + " - " + getLiftPositionError());

    if ((getLiftVelocity() < 100) && (getLiftPositionError() < Constants.LiftPositionTolerance))
      return false;
    else
      return true;
  }

  public static void updateLiftSimulation(){
    if (encoderCountSimulation < encoderTargetCountSimulation){
      //Need to move up. Fake an upward velocity
      liftVelocitySimulation = (encoderTargetCountSimulation - encoderCountSimulation);
      encoderCountSimulation = encoderCountSimulation + Constants.liftSpeedSimulation;
      //For simplicity at the moment immediately 'stop' when at the 'correct' height
      if (encoderCountSimulation > encoderTargetCountSimulation)
        encoderCountSimulation = encoderTargetCountSimulation;
    }
    else if (encoderCountSimulation > encoderTargetCountSimulation){
      //Need to move down. Fake a downward velocity
      liftVelocitySimulation = (encoderCountSimulation - encoderTargetCountSimulation);
      encoderCountSimulation = encoderCountSimulation - Constants.liftSpeedSimulation;
      //For simplicity at the moment immediately 'stop' when at the 'correct' height
      if (encoderCountSimulation < encoderTargetCountSimulation)
        encoderCountSimulation = encoderTargetCountSimulation;
    }
    else
      liftVelocitySimulation = 0;
  }

  @Override
  public void initDefaultCommand() {
  }
}
