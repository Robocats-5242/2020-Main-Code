package frc.robot.commands;

import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoCenterish{

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

    public static void start(){
        AlignWithTarget.alignWithTarget();
        Robot.shooter.autoShoot(2000);
        /*GyroRotate.gyroRotate(0);
        SmartDashboard.putString("Auto is...", "Moving backwards");
        DriveToPosition.driveToPosition(-66.5, Constants.AutoInSpeed, 0);
        GyroRotate.gyroRotate(-90);
        DriveToPosition.driveToPosition(60, Constants.AutoInSpeed, 0);
        GyroRotate.gyroRotate(-180);*/
        GyroRotate.gyroRotate(38);
        DriveToPosition.driveToPosition(110, Constants.AutoInSpeed, 0);
        GyroRotate.gyroRotate(180);
        DriveToPosition.driveToPosition(130, Constants.AutoInSpeed, 0);
        GyroRotate.gyroRotate(0);
        DriveToPosition.driveToPosition(196.5, Constants.AutoInSpeed, 0);
        GyroRotate.gyroRotate(10);
        AlignWithTarget.alignWithTarget();
        Robot.shooter.autoShoot(2000);
    }
}