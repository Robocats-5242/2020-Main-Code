package frc.robot.commands;

import edu.wpi.first.wpilibj.command.*;

public class DriveAndAlign extends CommandGroup{

    public DriveAndAlign(double distanceInches, double speed, double stopDistance){
        addSequential(new DriveToPosition(distanceInches, speed, stopDistance));
        addSequential(new AlignWithTarget());
    }
}