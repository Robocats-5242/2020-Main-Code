package frc.robot.commands;

import frc.robot.*;

public class AutoGen{

    public void start(){
        AlignWithTarget.alignWithTarget();
        Robot.shooter.autoShoot(3000);
        DriveToPosition.driveToPosition(20, Constants.AutoInSpeed, 0);
    }
}