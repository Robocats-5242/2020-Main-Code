package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;

public class AutoCenterishTraject{
    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(60 * 0.0254, 10 * 0.0254);

        String trajectoryJSON = "paths/Centrish.wpilib.json";
        Trajectory trajectory;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }       
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        Robot.driveTrain::getPose,
        new RamseteController(),
        Robot.driveTrain.kinematics,
        Robot.driveTrain::getWheelSpeeds,
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        Robot.driveTrain
    );
    }
}
