package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AutoShoot extends Command{
    
    boolean doneShooting = false;
    int timeoutLoops;

    public AutoShoot(int timeoutMs){
        requires(Robot.shooter);
        timeoutMs /= 20;
        timeoutLoops = timeoutMs;
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.setAutoFlag(true);
        Robot.shooter.shoot(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(timeoutLoops <= 0) doneShooting = true;
        timeoutLoops --;
    }

    @Override
    protected boolean isFinished(){
        return doneShooting;
    }
    protected void end(){
        Robot.driveTrain.setAutoFlag(true);
        Robot.shooter.shoot(false);
    }

    protected void interrupted(){

    }
}