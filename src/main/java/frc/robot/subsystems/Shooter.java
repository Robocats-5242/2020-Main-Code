package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

import frc.robot.Robot;
import frc.robot.commands.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {

    private CANSparkMax shooter1;
    private TalonFX shooterFalcon1;
    private TalonFX shooterFalcon2; 
    private CANPIDController pidController;
    private CANEncoder encoder;
    double setPoint = 0;



    public Shooter(){
        /*shooter1 = new CANSparkMax(Constants.CANShooter, MotorType.kBrushless);
        shooter1.restoreFactoryDefaults();
        pidController = shooter1.getPIDController();
        encoder = shooter1.getEncoder();
        setPoint = 0;

        shooter1.setInverted(true);
        pidController.setP(Constants.shooterkP);
        pidController.setI(Constants.shooterkI);
        pidController.setD(Constants.shooterkD);
        pidController.setIZone(Constants.shooterkIz);
        pidController.setFF(Constants.shooterkFF);
        pidController.setOutputRange(Constants.shooterkMinOutput, Constants.shooterkMaxOutput);
        pidController.setSmartMotionMaxVelocity(Constants.maxShooterRPM, 0);
        pidController.setSmartMotionMaxAccel(Constants.shooterAccel, 0);

        martDashboard.putNumber("Shooter P Gain", kP);
        SmartDashboard.putNumber("Shooter I Gain", kI);
        SmartDashboard.putNumber("Shooter D Gain", kD);
        SmartDashboard.putNumber("Shooter I Zone", kIz);
        SmartDashboard.putNumber("Shooter Feed Forward", kFF);
        SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
        SmartDashboard.putNumber("Shooter Min Output", kMinOutput);
        SmartDashboard.putNumber("Shooter Acceleration", accel);*/

        shooterFalcon1 = new TalonFX(20);
        shooterFalcon2 = new TalonFX(21);

        shooterFalcon1.configFactoryDefault();
        shooterFalcon2.configFactoryDefault();

        shooterFalcon1.configNeutralDeadband(0.001);
        shooterFalcon2.configNeutralDeadband(0.001);

        shooterFalcon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
        shooterFalcon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);

        shooterFalcon1.configNominalOutputForward(0, Constants.kTimeoutMs);
        shooterFalcon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
        shooterFalcon2.configNominalOutputForward(0, Constants.kTimeoutMs);
        shooterFalcon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
        shooterFalcon1.configPeakOutputForward(1, Constants.kTimeoutMs);
        shooterFalcon1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        shooterFalcon2.configPeakOutputForward(1, Constants.kTimeoutMs);
        shooterFalcon2.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        shooterFalcon1.setInverted(true);
        shooterFalcon2.setInverted(false);

        shooterFalcon1.config_kF(0, Constants.shooterkF, Constants.kTimeoutMs);
        shooterFalcon1.config_kP(0, Constants.shooterkP, Constants.kTimeoutMs);
        shooterFalcon1.config_kI(0, Constants.shooterkI, Constants.kTimeoutMs);
        shooterFalcon1.config_kD(0, Constants.shooterkD, Constants.kTimeoutMs);
        shooterFalcon2.config_kF(0, Constants.shooterkF, Constants.kTimeoutMs);
        shooterFalcon2.config_kP(0, Constants.shooterkP, Constants.kTimeoutMs);
        shooterFalcon2.config_kI(0, Constants.shooterkI, Constants.kTimeoutMs);
        shooterFalcon2.config_kD(0, Constants.shooterkD, Constants.kTimeoutMs);


    }

    public void shoot(){
        /*double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        accel = SmartDashboard.getNumber("Acceleration", 0);

        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }
        if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }
        pidController.setSmartMotionMaxAccel(accel, 0);*/
            if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonY)){
                //setPoint = Constants.maxShooterSpeed * Constants.maxShooterRPM;
                setPoint = Constants.maxShooterRPM * Constants.maxShooterSpeed * 2048 / 600;
            }else if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonX)){
                //setPoint = 0;
                setPoint = 0;
            }

        shooterFalcon1.set(TalonFXControlMode.Velocity, setPoint);
        shooterFalcon2.set(TalonFXControlMode.Velocity, setPoint);
        /*pidController.setReference(setPoint, ControlType.kSmartVelocity);
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());*/
    }
    public void initDefaultCommand(){

    }

    public void autoShoot(int msWait){
        int timeoutLoop = msWait * 20;
        //setPoint = Constants.maxShooterSpeed * Constants.maxShooterRPM;
        //pidController.setReference(setPoint, ControlType.kSmartVelocity);
        setPoint = Constants.maxShooterRPM * Constants.maxShooterSpeed * 2048 / 600;
        shooterFalcon1.set(TalonFXControlMode.Velocity, setPoint);
        shooterFalcon2.set(TalonFXControlMode.Velocity, setPoint);
        try{
            Thread.sleep(1500);
        } catch(InterruptedException ex){
            
        }

        Robot.hopper.setHopper(Constants.HopperSpeed);
        while(timeoutLoop > 0){
            shooterFalcon1.set(TalonFXControlMode.Velocity, setPoint);
            shooterFalcon2.set(TalonFXControlMode.Velocity, setPoint);
            //pidController.setReference(setPoint, ControlType.kSmartVelocity);
            //SmartDashboard.putNumber("SetPoint", setPoint);
            //SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
            //SmartDashboard.putNumber("Loop Count Shooter", timeoutLoop);
            timeoutLoop --;
        }
        setPoint = 0;
        shooterFalcon1.set(TalonFXControlMode.Velocity, setPoint);
        shooterFalcon2.set(TalonFXControlMode.Velocity, setPoint);
        //pidController.setReference(setPoint, ControlType.kSmartVelocity);
        Robot.hopper.setHopper(0);
    }
}