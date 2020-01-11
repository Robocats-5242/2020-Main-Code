/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class Constants {
	//CAN bus channels for devices
	public static final int    CANLeftFrontMasterController    = 6;
	public static final int    CANRightFrontMasterController   = 7;
	public static final int    CANLeftFrontFollowerController  = 11;
	public static final int    CANRightFrontFollowerController = 9;
	public static final int    CANLiftMasterController         = 10;
	public static final int    CANLiftFollowerController       = 12;
	public static final int    CANIntakeController             = 8;
	public static final int    CANClimberController            = 5;
	public static final int    CANCrawlerController            = 1;  //change for actual id and check for robot 2

	//Pneumatic constants
	public static final int     PneuStroke1Channel             = 0;
	public static final int     PneuStroke2Channel             = 1;
	public static final boolean PneuLiftOut                    = true;//ToDo : Check correct order
	public static final boolean PneuLiftIn                     = false;

	//Digital IO channels
	public static final int    DigUltrasonicPingChannel        = 1;
	public static final int    DigUltrasonicEchoChannel        = 2;

	//Drive train constants
	public static final int    WheelCountsPerRev               = 4096;
	public static final double WheelDiameter                   = 6;//6 inch diameter
	public static final double WheelCircumference              = Math.PI*WheelDiameter;
	public static final double WheelTicksPerInch               = WheelCountsPerRev / WheelCircumference;
	public static final int    SpeedMaxTicksPer100mS           = 8000;//ToDo : Need to measure
	public static final double DriveStraightPGain              = 0.1;//ToDo : Need to tune
	public static final double MaxAllowableTargetError         = 0.03;

	//Lift constants
	public static final double LiftSprocketCircumference       = 4;//16 teeth spaced at 1/4 inch
	public static final double LiftCountsPerRev                = 4096;
	public static final double LiftTicksPerInch                = (LiftCountsPerRev / LiftSprocketCircumference) / 2;//Divided by 2 because of two-stage lift
	public static final double LiftMaxHeight                   = 81;
	public static final double LiftPositionTolerance           = 2;
	public static final double LiftManualSpeed                 = 4.0/50;//4 inches per second since loops at 50 itterations / second

	//Auto constants
	public static final double AutoInSpeed                = 0.4;//Speed at which the semi-auto moves towards the target
	public static final double AutoOutSpeed               = 0.6;//Speed at which the semi-auto moves away from the target
	public static final double AutoStopFromDistanceCargo  = 6.0;//Distance from the target to stop for cargo (account for the hook)
	public static final double AutoStopFromDistanceHatch  = 4.0;//Distance from the target to stop for hatches (needs to be inside target)
	public static final double AutoStopMaxDistance        = 30.0;//Max distance to try to travel when moving towards target
	public static final double AutoDefaultLiftHeight      = 30.0;//Height to move lift to after picking up/depositing something
	public static final double AutoBackoffDistance        = 12.0;//Distance to back up after picking up/depositing something
	public static final double AutoHabCreepSpeed          = 0.3;//Speed at which the drive train runs when climbing the hab

	//Cargo constants
	public static final double CargoDepositLowHeight       = 25.0;
	public static final double CargoDepositMidHeight       = 53.0;
	public static final double CargoDepositHighHeight      = 76.0;
	public static final double CargoDepositRoverHeight     = 35.0;
	public static final double CargoRetrieveDepotHeight    = 40.0;
	public static final double CargoRetrieveGroundHeight   = 2.0;
	public static final int    CargoRetrieveLocationGround = 0;//Enumeration for different locations
	public static final int    CargoRetrieveLocationDepot  = 1;//Enumeration for different locations
	public static final double CargoEjectDelay             = 0.1;

	//Intake constants
	public static final int     IntakeStateOff   = 0;
	public static final int     IntakeStateHold  = 1;
	public static final int     IntakeStateIn    = 2;
	public static final int     IntakeStateOut   = 3;
	public static final double  IntakeInSpeed    = 0.75;
	public static final double  IntakeEjectSpeed = -1;
	public static final double  IntakeHoldSpeed  = 0.25;
	public static final double  DriveHeight      = 5.0;

	//Hatch constants
	//Heights are the height at which the robot should be at to be able to go through center ToDo : THESE HEIGHTS NEED TO BE CHANGED
	//decreased by 2in
	//give the hatch panel pick up about 1 in to 1.5in of clearance at the top of the panel 
	public static final double  HatchDepositLowHeight       = 3.0;
	public static final double  HatchDepositMidHeight       = 32.0;
	public static final double  HatchDepositHighHeight      = 61.0;
	public static final double  HatchDepositRoverHeight     = 3.0;
	public static final double  HatchRetrieveDepotHeight    = 4.5;
	public static final double  HatchDepositDelta           = 4.0;//Distance to move down in order to unhook when depositing
	public static final boolean HatchDownState              = false;//Flag to note if hook is up or down
	public static final boolean HatchUpState                = true;

	//Controller slots
	public static final int   MainController               = 0;
	public static final int   ButtonController             = 1;

	//Button allocations
//	public static final int   TestButton                   = 1;
	public static final int   HatchLiftButton              = 1;
	public static final int   LiftMoveUpButton             = 2;
	public static final int   CargoIntakeOutButton         = 3;
	public static final int   HatchDropButton              = 4;
	public static final int   SetDriveHeightButton         = 5;
	public static final int   CargoIntakeInButton          = 6;
	public static final int   CargoDepositLowButton        = 7;
	public static final int   CargoDepositMidButton        = 8;
	public static final int   CargoDepositHighButton       = 9;
	public static final int   CargoDepositRoverButton      = 10;
	public static final int   HatchDepositLowButton        = 11;
//	public static final int   HatchDepositRoverButton      = 11;
	public static final int   HatchDepositMidButton        = 12;
	public static final int   HatchDepositHighButton       = 13;
	public static final int   CargoRetrieveGroundButton    = 14;
	public static final int   HatchRetrieveDepotButton     = 15;
	public static final int   CargoRetrieveDepotButton     = 16;
	
	//Controller button allocations
	public static final int   LiftExtendButton             = 1;
	public static final int   LiftRetractButton            = 2;
	public static final int   LiftUpButton                 = 3;
	public static final int   LiftDownButton               = 4;

	//XBox raw button mappings
	public static final int   XBoxButtonX                 = 3;
	public static final int   XBoxButtonY                 = 4;
	public static final int   XBoxButtonA                 = 1;
	public static final int   XBoxButtonB                 = 2;
	public static final int   XBoxButtonStickLeft         = 9;
	public static final int   XBoxButtonStickRight        = 10;
	public static final int   XBoxButtonTriggerLeft       = 5;
	public static final int   XBoxButtonTriggerRight      = 6;
	public static final int   XBoxButtonHome              = 7;
	public static final int   XBoxButtonMenu              = 8; 
	
	public static final int   HabClimbButton              = XBoxButtonMenu;    //climber articulation = LogitechButtonStart
	public static final int   HabRetractButton            = XBoxButtonHome;    //climber articulation = LogitechButtonStart

	//Lift PID parameters
	public static final int     LiftkPIDkSlotIdx     = 0;
	public static final int     LiftkkPIDLoopIdx     = 0;
	public static final int     LiftkTimeoutMs       = 30;
	public static final boolean LiftkSensorPhase     = true;
	public static final boolean LiftkMotorInvert     = true;
	//kp, ki, kd, kf, izone, peak output
	//ToDo : Need to set these parameters
	public static final double  LiftPIDkP             = 0.1;
	public static final double  LiftPIDkI             = 0.0;
	public static final double  LiftPIDkD             = 0.0;
	public static final double  LiftPIDkF             = 0.0;
	public static final double  LiftPIDizone          = 0.0;
	public static final double  LiftPIDpeakoutputUp   = 1.0;
	public static final double  LiftPIDpeakoutputDown = 0.4;
	public static final int     LiftPIDmaxerror       = 125;       //500 ticks is 1 inch

	//Drivetrain PID parameters
	//Joystick parameters
	public static final double  JoystickAccelleration = 0.07;
	public static final double  JoystickDecelleration = 0.1;
	public static final double  CrawlSpeed            = 0.15;

	//PID parameters
	public static final int     DrivekPIDkSlotIdx     = 1;
	public static final int     DrivekkPIDLoopIdx     = 0;
	public static final int     DrivekTimeoutMs       = 30;
	public static final boolean DrivekSensorPhase     = true;
	public static final boolean DrivekMotorInvert     = true;
	//kp, ki, kd, kf, izone, peak output
	//ToDo : Need to set these parameters
	public static final double  DrivePIDkP            = 0.3;
	public static final double  DrivePIDkI            = 0.0;
	public static final double  DrivePIDkD            = 0.0;
	public static final double  DrivePIDkF            = 0.0;
	public static final double  DrivePIDizone         = 0.0;
	public static final double  DrivePIDpeakoutput    = 1.0;
	public static final int     DrivePIDmaxerror      = 0;

	//Simulation parameters
	public static final int liftSpeedSimulation       = 190;

	//Climber PID parameters
	public static final int     ClimberkTimeoutMs       = 30;
	public static final double  ClimberPIDpeakoutput    = 1.0;
	public static final double  ClimberTargetZ          = 0.1;
	public static final double  HabCrawlSpeed           = 1.0;
	public static final double  ClimberRetractSpeed     = 1.0;//ToDo : Check polarity

	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
//	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
//	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	//public static final int kTimeoutMs = 30;
	
	/* Choose so that Talon does not report sensor out of phase */
	//public static boolean kSensorPhase = true;

	/**
	 * Choose based on what direction you want to be positive,
	 * this does not affect motor invert. 
	 */
	//public static boolean kMotorInvert = true;

	/**
	 * Gains used in Positon Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
	//static final Gains kGains = new Gains(0.02, 0.0, 1.0, 0.0, 0, 1.0);
	//public static final Gains kGains = new Gains(0.3, 0.0, 1.0, 0.0, 0, 1.0);
}
