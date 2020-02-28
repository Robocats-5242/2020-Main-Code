package frc.robot.commands;

import edu.wpi.first.wpilibj.command.*;
import frc.robot.Constants;

public class AutoCenterish extends CommandGroup{

    public AutoCenterish (){
        //addSequential(new DriveToPosition(-14, Constants.AutoInSpeed, 0));
        /*addSequential(new AlignWithTarget());
        addSequential(new AutoShoot(2000));
        addSequential(new GyroRotate(0));
        addSequential(new DriveToPosition(-66.5, Constants.AutoInSpeed, 0));
        addSequential(new GyroRotate(-90)); //Rotate via gyro to -90
        addSequential(new DriveToPosition(67, Constants.AutoInSpeed, 20));
        addSequential(new GyroRotate(-180));//Rotate via gyro to -180
        //Extend intake
        addSequential(new DriveToPosition(130, Constants.AutoInSpeed, 0));
        //Retract intake
        addSequential(new GyroRotate(0));//Rotate to 0
        addSequential(new DriveToPosition(196.5, Constants.AutoInSpeed, 0));
        addSequential(new AlignWithTarget());
        addSequential(new AutoShoot(2000));*/
    }
}