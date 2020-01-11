/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
public class LiftAndDepositCargo extends CommandGroup {
  /**
   * Deposit cargo to selected height
   *   Align with the guide line
   *   Move the lift to the correct height
   *   Drive straight to the correct distance from the wall
   *   Turn on the intake and wait for a ball to be ejected
   *   Back away from the wall
   *   Set the lift to the default height
   */
  public LiftAndDepositCargo(double height) {
    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
     //addSequential(new FindLine());
     addSequential(new LiftToHeight(height, 0, true));//Move the lift to the selected height and wait until finished
     //addSequential(new DriveToPosition(24, Constants.AutoInSpeed, 4));//Move to within 4 inches of the target but a max of 24 inches
     //addSequential(new HandleCargo(Constants.IntakeOut, true, true));//Eject the cargo, wait until ejected
     //addSequential(new DriveToPosition(-Constants.AutoBackoffDistance, Constants.AutoOutSpeed, 10000));
     //addSequential(new LiftToHeight(Constants.AutoDefaultLiftHeight,0, false));
     addSequential(new WaitButtonsReleased());
  }
}
