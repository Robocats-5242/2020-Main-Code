package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.*;
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

    //static VictorSP shooter2;
    private CANSparkMax shooter1;
    private CANPIDController pidController;
    private CANEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, accel;
    private double setPoint = 0;


    public Shooter(){
        //shooter2 = new VictorSP(1);
        shooter1 = new CANSparkMax(Constants.CANShooter, MotorType.kBrushless);
        shooter1.restoreFactoryDefaults();
        pidController = shooter1.getPIDController();
        encoder = shooter1.getEncoder();

        kP = 7e-5; 
        kI = 2e-7;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
        accel = 10000;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        pidController.setSmartMotionMaxVelocity(maxRPM, 0);

        SmartDashboard.putNumber("Shooter P Gain", kP);
        SmartDashboard.putNumber("Shooter I Gain", kI);
        SmartDashboard.putNumber("Shooter D Gain", kD);
        SmartDashboard.putNumber("Shooter I Zone", kIz);
        SmartDashboard.putNumber("Shooter Feed Forward", kFF);
        SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
        SmartDashboard.putNumber("Shooter Min Output", kMinOutput);
        SmartDashboard.putNumber("Shooter Acceleration", accel);
    }

    public void shoot(){
        double p = SmartDashboard.getNumber("P Gain", 0);
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
        pidController.setSmartMotionMaxAccel(accel, 0);
        if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonX)){
            //shooter2.set(.5);
            setPoint = 0.4 * maxRPM;
        }else if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonY)){
            setPoint = 0.6 * maxRPM;
            //shooter2.set(.5);
        }else if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonB)){
            setPoint = 0;
            //shooter2.set(0);
        }

        pidController.setReference(setPoint, ControlType.kSmartVelocity, 0);
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
    }
    public void initDefaultCommand(){

    }
}