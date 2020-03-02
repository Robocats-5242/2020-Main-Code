package frc.robot.commands;

import frc.robot.*;

public class AutoGen{

    public static void start(){
        AlignWithTarget.alignWithTarget();
        Robot.shooter.autoShoot(3000);
        DriveToPosition.driveToPosition(-40, Constants.AutoInSpeed, 0);
    }
}