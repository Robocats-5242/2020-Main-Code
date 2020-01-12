/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
//import javax.swing.text.StyleContext.SmallAttributeSet;
//import java.net.Socket;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.Robot;

public class DriveTrain extends Subsystem {
  /*TalonSRX rightFront; //2019 DriveTrain
  TalonSRX rightFollower;
  TalonSRX leftFront;
  TalonSRX leftFollower;*/
  CANSparkMax rightFront;
  CANSparkMax rightFollower;
  CANSparkMax leftFront;
  CANSparkMax leftFollower;
  CANEncoder rightEncoder;
  CANEncoder leftEncoder;
  CANPIDController rightPidController;
  CANPIDController leftPidController;
  Ultrasonic ultrasonicSensor;
  private double leftCurrentPercentJoystick = 0.0;
  private double rightCurrentPercentJoystick = 0.0;
  private double leftCurrentPercentAuto = 0.0;
  private double rightCurrentPercentAuto = 0.0;
  private double leftCurrentPercent = 0.0;
  private double rightCurrentPercent = 0.0;
  private String CommandName = "DriveTrain";
  private int leftEncoderSimulation = 1021210;//Set to random value to check that we actually reset correctly
  private int rightEncoderSimulation = -1213310;
  private double ultrasonicRangeSimulation = 0;
  private double leftSpeedSimulation = 0;
  private double rightSpeedSimulation = 0;
  private boolean autoFlag = false;

  private void driveTrainInit(){
    rightFront    = new CANSparkMax(Constants.CANRightFrontMasterController, MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.CANRightFrontFollowerController, MotorType.kBrushless);
    leftFront     = new CANSparkMax(Constants.CANLeftFrontMasterController, MotorType.kBrushless);
    leftFollower  = new CANSparkMax(Constants.CANLeftFrontFollowerController, MotorType.kBrushless);
    rightEncoder = new CANEncoder(rightFront);
    leftEncoder = new CANEncoder(leftFront);
    rightPidController= new CANPIDController(rightFront);
    leftPidController = new CANPIDController(leftFront);

    //Reset all factory defaults
    /*rightFront.configFactoryDefault(); //2019 Factory Default
    rightFollower.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftFollower.configFactoryDefault();*/
     //Configure drive train
    //Make constants different to those used for the lift
    //rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.DrivekkPIDLoopIdx, Constants.DrivekTimeoutMs);
    rightPidController.setFeedbackDevice(rightEncoder);
    //rightFront.setSensorPhase(Constants.DrivekSensorPhase);
    rightFront.setInverted(Constants.DrivekMotorInvert);
    //rightFront.configNominalOutputForward(0, Constants.DrivekTimeoutMs);
    //rightFront.configNominalOutputReverse(0, Constants.DrivekTimeoutMs);
    //rightFront.configPeakOutputForward(Constants.DrivePIDpeakoutput, Constants.DrivekTimeoutMs);
    //rightFront.configPeakOutputReverse(-Constants.DrivePIDpeakoutput, Constants.DrivekTimeoutMs);
    rightPidController.setOutputRange(-1, 1, 0);
    //rightFront.configAllowableClosedloopError(Constants.DrivePIDmaxerror, Constants.DrivekkPIDLoopIdx, Constants.DrivekTimeoutMs);
    rightPidController.setSmartMotionAllowedClosedLoopError(Constants.DrivePIDmaxerror, 0);
    //rightFront.config_kF(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkF, Constants.DrivekTimeoutMs);
    rightPidController.setFF(Constants.DrivePIDkF);
    //rightFront.config_kP(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkP, Constants.DrivekTimeoutMs);
    rightPidController.setP(Constants.DrivePIDkP);
    //rightFront.config_kI(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkI, Constants.DrivekTimeoutMs);
    rightPidController.setI(Constants.DrivePIDkI);
    //rightFront.config_kD(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkD, Constants.DrivekTimeoutMs);
    rightPidController.setD(Constants.DrivePIDkD);
    /* Set the quadrature (relative) sensor to match absolute */
    //rightFront.setSelectedSensorPosition(0, Constants.DrivekkPIDLoopIdx, Constants.DrivekTimeoutMs);

    //leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.DrivekkPIDLoopIdx, Constants.DrivekTimeoutMs);
    leftPidController.setFeedbackDevice(rightEncoder);
    //leftFront.setSensorPhase(Constants.DrivekSensorPhase);
    leftFront.setInverted(!Constants.DrivekMotorInvert);
    //leftFront.configNominalOutputForward(0, Constants.DrivekTimeoutMs);
    //leftFront.configNominalOutputReverse(0, Constants.DrivekTimeoutMs);
    //leftFront.configPeakOutputForward(Constants.DrivePIDpeakoutput, Constants.DrivekTimeoutMs);
    //leftFront.configPeakOutputReverse(-Constants.DrivePIDpeakoutput, Constants.DrivekTimeoutMs);
    leftPidController.setOutputRange(-1, 1, 0);
    //leftFront.configAllowableClosedloopError(Constants.DrivePIDmaxerror, Constants.DrivekkPIDLoopIdx, Constants.DrivekTimeoutMs);
    leftPidController.setSmartMotionAllowedClosedLoopError(Constants.DrivePIDmaxerror, 0);
    //leftFront.config_kF(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkF, Constants.DrivekTimeoutMs);
    leftPidController.setFF(Constants.DrivePIDkF);
    //leftFront.config_kP(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkP, Constants.DrivekTimeoutMs);
    leftPidController.setP(Constants.DrivePIDkP);
    //leftFront.config_kI(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkI, Constants.DrivekTimeoutMs);
    leftPidController.setI(Constants.DrivePIDkI);
    //leftFront.config_kD(Constants.DrivekkPIDLoopIdx, Constants.DrivePIDkD, Constants.DrivekTimeoutMs);
    leftPidController.setD(Constants.DrivePIDkD);

    //Set rampe rate. ToDo : Dynamically change PID in the joystick controlled code to effectively set different forward and backwards rates. Don't forget to configure for semi auto actions though !!
    //rightFront.configOpenloopRamp(.3, 1000);
    //leftFront.configOpenloopRamp(.3, 1000);
    rightFront.setOpenLoopRampRate(.3);
    leftFront.setOpenLoopRampRate(.3);

    /* set up followers */
    rightFollower.follow(rightFront);
    leftFollower.follow(leftFront);

    /*rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFront.setSensorPhase(true);
    leftFront.setSensorPhase(true);*/    

    //leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 6,  Constants.DrivekTimeoutMs);
    //rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 7 ,  Constants.DrivekTimeoutMs);
    //leftFront.setNeutralMode(NeutralMode.Brake);
    //rightFront.setNeutralMode(NeutralMode.Brake);
    //leftFollower.setNeutralMode(NeutralMode.Brake);
    //rightFollower.setNeutralMode(NeutralMode.Brake);
    leftFront.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    //Configure ultrasonic range finder
//    ultrasonicSensor = new Ultrasonic(Constants.DigUltrasonicPingChannel, Constants.DigUltrasonicEchoChannel, Unit.kMillimeters);
    ultrasonicSensor = new Ultrasonic(Constants.DigUltrasonicPingChannel, Constants.DigUltrasonicEchoChannel);

  }

  public DriveTrain(){
    //Drive train constructor
    Robot.logMessage(CommandName, "constructor");
    //Initialize the drive train
    if (Robot.isReal() && Robot.useHardware()){
      driveTrainInit();
    }
    resetEncoders();
  }

  public void resetEncoders(){
    //ToDo : Check the parameters. They are supposed to be the count, PID & timeout values  the CAN ID
//    _leftFront.setSelectedSensorPosition(6,1,1);
//    _rightFront.setSelectedSensorPosition(8,1,1);
    if (Robot.isReal() && Robot.useHardware()){
      //leftFront.setSelectedSensorPosition(0,0,Constants.DrivekTimeoutMs);
      //rightFront.setSelectedSensorPosition(0,0,Constants.DrivekTimeoutMs);
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }
    else{
      leftEncoderSimulation = 0;
      rightEncoderSimulation = 0;
    }
  }

  private double getLeftEncoderTicks(){
    if (Robot.isReal() && Robot.useHardware())
      //return leftFront.getSelectedSensorPosition();
      return leftEncoder.getPosition();
    else
      return leftEncoderSimulation;
  }

  private double getRightEncoderTicks(){
    if (Robot.isReal() && Robot.useHardware())
      //return rightFront.getSelectedSensorPosition();
      return rightEncoder.getPosition();
    else
      return rightEncoderSimulation;
  }

  public double getLeftEncoderInches(){
    return getLeftEncoderTicks() / Constants.WheelTicksPerInch;
  }

  public double getRightEncoderInches(){
    return getRightEncoderTicks() / Constants.WheelTicksPerInch;
  }

  public void setSpeedPercentJoystick(double leftSpeed, double rightSpeed){
    leftCurrentPercentJoystick = leftSpeed;
    rightCurrentPercentJoystick = rightSpeed;
    //setSpeedRaw(leftSpeed * Constants.SpeedMaxTicksPer100mS, rightSpeed * Constants.SpeedMaxTicksPer100mS);
  }

  public void setSpeedPercentAuto(double leftSpeed, double rightSpeed){
    leftCurrentPercentAuto = leftSpeed;
    rightCurrentPercentAuto = rightSpeed;
    //setSpeedRaw(leftSpeed * Constants.SpeedMaxTicksPer100mS, rightSpeed * Constants.SpeedMaxTicksPer100mS);
  }

  public void updateDriveTrain(){
    if (autoFlag == true){
      leftCurrentPercent = leftCurrentPercentAuto;
      rightCurrentPercent = rightCurrentPercentAuto;
    }
    else{
      leftCurrentPercent = leftCurrentPercentJoystick;
      rightCurrentPercent = rightCurrentPercentJoystick;
    }
    setSpeedRaw(leftCurrentPercent * Constants.SpeedMaxTicksPer100mS, rightCurrentPercent * Constants.SpeedMaxTicksPer100mS);
  }

  public double getLeftSpeedPercent(){
    return leftCurrentPercent;
  }

  public double getRightSpeedPercent(){
    return rightCurrentPercent;
  }
/*
  public void driveDistanceStraight(double distance) {
    //Drive straight for the specified distance at the default speed, no ultrasonic override
    driveDistanceStraight(distance, 0.5, -1000);
  }

  public void driveDistanceStraight(double distance, double speed) {
    //Drive straight for the specified distance at the specified speed, no ultrasonic override
    driveDistanceStraight(distance, 0.5, -1000);
  }
*/
  public double getUltrasonicRange(){
    if (Robot.isReal() && Robot.useHardware())
      return ultrasonicSensor.getRangeInches();
    else {
      return ultrasonicRangeSimulation;
    }
  }

  public void setUltrasonicValueSimulation(double fakeInches){
    ultrasonicRangeSimulation = fakeInches;
  }

  private void setSpeedRaw(double leftSpeed, double rightSpeed){
    //Speed is ticks per 100mS ?
    if (Robot.isReal() && Robot.useHardware()){
      leftFront.set(leftSpeed);
      rightFront.set(rightSpeed);
    }
    else
    {
      leftSpeedSimulation = leftSpeed;
      rightSpeedSimulation = rightSpeed;
    }
  }

  @Override
  public void initDefaultCommand() {
    // If not doing anything else then drive with the joysticks
    //setDefaultCommand(new DrivesWithJoysticks());//ToDo : Move to xxxLoops in robot to allow merged auto-joystick
  }

  public void updateDrivetrainSimulation(){
    leftEncoderSimulation = (int)(leftEncoderSimulation + (leftSpeedSimulation / 50));
    rightEncoderSimulation = (int)(rightEncoderSimulation + (rightSpeedSimulation / 50));
    setUltrasonicValueSimulation(getUltrasonicRange() + ((leftSpeedSimulation / 50) + (rightSpeedSimulation / 50)));//ToDo : Fctor needs to be tuned to make reasonable
  }

  public void setAutoFlag(boolean state){
    autoFlag = state;
  }

}
