package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.VictorSP;

import frc.robot.Robot;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Hopper extends Subsystem {
    VictorSP hopper1;
    VictorSP hopper2;

    public Hopper(){
        hopper1 = new VictorSP(Constants.PWMHopper1);
        hopper2 = new VictorSP(Constants.PWMHopper2);

        hopper1.setInverted(false);
        hopper2.setInverted(false);
    }

    public void updateHopper(){
        if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonA)){
            setHopper(Constants.hopperSpeed);
        }
        if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonB)){
            setHopper(0);
        }
    }

    public void setHopper(double power){
        hopper1.set(power);
        hopper2.set(power);
    }

    public void initDefaultCommand(){

    }
}