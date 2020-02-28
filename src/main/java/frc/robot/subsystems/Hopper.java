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
    }

    public void updateHopper(){
        if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonA)){
            hopper1.set(Constants.hopperSpeed);
            hopper2.set(Constants.hopperSpeed);
        }
        if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonB)){
            hopper1.set(0);
            hopper2.set(0);
        }
    }

    public void initDefaultCommand(){

    }
}