/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.Faults;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.Ultrasonic.Unit;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DrivesWithJoysticks;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort;

//https://www.bing.com/videos/search?q=best+tom+and+jerry&view=detail&mid=ADE99A515A0765A73415ADE99A515A0765A73415&FORM=VIRE 
public class Robot extends TimedRobot {
  public static OI operatorInterface;
  public static Vision visionSystem;
  //Command m_autonomousCommand;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  Command driveWithJoystick;
  //Command liftToPosition;
  public static Hand leftStick = Hand.kLeft;
  public static Hand rightStick = Hand.kRight;

  public static DriveTrain driveTrain;
  public static Intake intakeSystem;
  public static Lift liftSystem;
  public static Pneumatics pneumaticSystem;
  public static RobotAccelerometer imu;
  public static DrivesWithJoysticks driveIntake;
  public static Climber habClimber;

  Faults _faults_L = new Faults();
  Faults _faults_R = new Faults();

  SerialPort visionPort = null;
  int loopCount = 0;

  private void updateSmartDashboard(){
    //Historically we have found that if items are not added 
    SmartDashboard.putNumber("Left power", driveTrain.getLeftSpeedPercent());
    SmartDashboard.putNumber("Right power", driveTrain.getRightSpeedPercent());
    SmartDashboard.putNumber("LiftPosition", liftSystem.getLiftPositionInches());
    SmartDashboard.putNumber("Left dist.", driveTrain.getLeftEncoderInches());
    SmartDashboard.putNumber("Right dist.", driveTrain.getRightEncoderInches());
    SmartDashboard.putBoolean("Cargo in", intakeSystem.isBallIn());
    SmartDashboard.putBoolean("Lift extend", pneumaticSystem.getPneumaticsState());
    SmartDashboard.putNumber("Intake power", intakeSystem.getIntakeSpeed());
    SmartDashboard.putNumber("Lift error", liftSystem.getLiftPositionErrorTotal());
    SmartDashboard.putNumber("Target error", visionSystem.getTargetError());

//    SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
//    SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
//    SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());
    
    SmartDashboard.putNumber("Accel-X", imu.getAccelX());
    SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
    SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());
    SmartDashboard.putNumber("Accel-Z Ave.", imu.getAccelZAverage());
    SmartDashboard.putNumber("Climber Power", habClimber.getClimberPower());
    SmartDashboard.putNumber("Crawler Power", habClimber.getCrawlerPower());

  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    liftSystem = new Lift();
    pneumaticSystem = new Pneumatics();
    intakeSystem = new Intake();
    driveWithJoystick = new DrivesWithJoysticks();
    //OI must come after subsystems since it references commands which in turn reference sub-systems
    visionSystem = new Vision();
    imu = new RobotAccelerometer();
    habClimber = new Climber();
    operatorInterface = new OI();

    //Make a note of the current angle of the accelerometer
    imu.captureOffset();

    updateSmartDashboard();
    
//    ADIS16470_IMU imu = new ADIS16470_IMU();
//    vision = new Vision();
//    ultrasonicSensorVision = new UltrasonicSensor();
//    ultrasonicSensorDrive = new UltrasonicSensorDrive();
  
     // pneumaticsSmash.setClosedLoopControl(true);
    
     if (Robot.isReal() == true){
      CameraServer.getInstance().startAutomaticCapture(0);
      //CameraServer.getInstance().startAutomaticCapture(1);
     }
 
  }

  private void updateSimulations(){
    liftSystem.updateLiftSimulation();
    intakeSystem.updateIntakeSimulation();
    driveTrain.updateDrivetrainSimulation();
  }
  
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //Use this for simulation updates or when on RoboRio but no hardware connected
//    if (Robot.isSimulation() || ((Robot.isReal() && (Robot.useHardware() == false))))
      updateSimulations();
    //RobotAccelerometer.updateAveraging();
    Robot.imu.updateAveraging();
    Robot.visionSystem.updateVision();
    updateSmartDashboard();
  }


  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();//ToDo : Do we really want the scheduler to run when disabled?
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      doActivePeriodic();
    }
 
    @Override
  public void teleopInit() {    
  } 

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {
    doActivePeriodic();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static void logMessage(String module, String message){
    System.out.println(module + " : " + message);
  }

  public static boolean useHardware(){
      return false;
  }

  public static boolean useJoysticks(){
    return true;
}

  private void doActivePeriodic(){
    //Since we don't have different auto vs teleop we should run the same events in both xxxPeriodic loops
    Scheduler.getInstance().run();
    DrivesWithJoysticks.updateDriveFromJoystick();
    driveTrain.updateDriveTrain();
    intakeSystem.updateIntake();
  }

 }
