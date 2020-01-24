package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.*;
import frc.robot.Robot;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {

    static VictorSP shooter1;
    static VictorSP shooter2;

    public Shooter(){
        shooter1 = new VictorSP(1);
        shooter2 = new VictorSP(2);
    }

    public void shootShooter(){
        if(Robot.operatorInterface.getControllerButtonState(Constants.XBoxButtonY)){
            shooter1.set(.5);
            shooter2.set(.5);
        }else{
            shooter1.set(0);
            shooter2.set(0);
        }
    }
    public void initDefaultCommand(){

    }
}