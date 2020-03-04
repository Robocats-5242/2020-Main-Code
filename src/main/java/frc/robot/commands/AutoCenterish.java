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
        SmartDashboard.putString("Auto is...", "Shooting");
        AlignWithTarget.alignWithTarget();
        Robot.shooter.autoShoot(2000);
        SmartDashboard.putString("Auto is...", "Turning 1");
        GyroRotate.gyroRotate(0);
        SmartDashboard.putString("Auto is...", "Moving backwards");
        DriveToPosition.driveToPosition(-66.5, Constants.AutoInSpeed, 0);
        SmartDashboard.putString("Auto is...", "Turning -90");
        GyroRotate.gyroRotate(-90);
        SmartDashboard.putString("Auto is...", "Moving forwards 1");
        DriveToPosition.driveToPosition(67, Constants.AutoInSpeed, 0);
        SmartDashboard.putString("Auto is...", "Turning -180");
        GyroRotate.gyroRotate(-180);
        SmartDashboard.putString("Auto is...", "Moving forwards 2");
        DriveToPosition.driveToPosition(130, Constants.AutoInSpeed, 0);
        SmartDashboard.putString("Auto is...", "Turning 0");
        GyroRotate.gyroRotate(-1);
        SmartDashboard.putString("Auto is...", "Moving forwards 3");
        DriveToPosition.driveToPosition(196.5, Constants.AutoInSpeed, 0);
        SmartDashboard.putString("Auto is...", "Shooting again");
        AlignWithTarget.alignWithTarget();
        Robot.shooter.autoShoot(2000);
        SmartDashboard.putString("Auto is...", "Done!");
    }
}