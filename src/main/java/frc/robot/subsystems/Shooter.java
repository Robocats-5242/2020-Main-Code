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

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {

    private CANSparkMax shooter1;
    private CANPIDController pidController;
    private CANEncoder encoder;
    double setPoint = 0;



    public Shooter(){
        shooter1 = new CANSparkMax(Constants.CANShooter, MotorType.kBrushless);
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

        /*SmartDashboard.putNumber("Shooter P Gain", kP);
        SmartDashboard.putNumber("Shooter I Gain", kI);
        SmartDashboard.putNumber("Shooter D Gain", kD);
        SmartDashboard.putNumber("Shooter I Zone", kIz);
        SmartDashboard.putNumber("Shooter Feed Forward", kFF);
        SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
        SmartDashboard.putNumber("Shooter Min Output", kMinOutput);
        SmartDashboard.putNumber("Shooter Acceleration", accel);*/
    }

    public void shoot(boolean autoShoot){
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
            if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonY)){
                setPoint = Constants.maxShooterSpeed * Constants.maxShooterRPM;
            }else if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonX)){
                setPoint = 0;
            }

        pidController.setReference(setPoint, ControlType.kSmartVelocity);
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
    }
    public void initDefaultCommand(){

    }

    public void autoShoot(int msWait){
        int timeoutLoop = msWait / 20;
        setPoint = Constants.maxShooterSpeed * Constants.maxShooterRPM;
        boolean doneShooting = false;
        pidController.setReference(setPoint, ControlType.kSmartVelocity);
        while(!doneShooting){
            SmartDashboard.putNumber("SetPoint", setPoint);
            SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
            if(timeoutLoop <= 0) doneShooting = true;
            timeoutLoop --;
        }
        setPoint = 0;
        pidController.setReference(setPoint, ControlType.kSmartVelocity);
    }
}