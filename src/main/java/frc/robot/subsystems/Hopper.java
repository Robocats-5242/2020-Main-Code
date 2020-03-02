package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        hopper1.setInverted(true);
        hopper2.setInverted(true);
    }

    public void updateHopper(){
        if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonA)){
            setHopper(Constants.HopperSpeed);
            SmartDashboard.putString("Hopper is...", "On");
        }
        if(Robot.operatorInterface.getControllerTriggerRightOp() > 0.05){
            setHopper(Constants.HopperSpeedReverse);
            SmartDashboard.putString("Hopper is...", "In Reverse");
        }
        if(Robot.operatorInterface.getControllerButtonStateOp(Constants.XBoxButtonB)){
            setHopper(0);
            SmartDashboard.putString("Hopper is...", "Off");
        }
    }

    public void setHopper(double power){
        hopper1.set(power);
        hopper2.set(power);
    }

    public void initDefaultCommand(){

    }
}