/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
//import frc.robot.commands.*;
//import frc.robot.*;
//import frc.robot.subsystems.*;
public class LiftAndRetrieveHatch extends CommandGroup {
  /**
   * Deposit cargo to selected height
   *   Align with the guide line
   *   Move the lift to the correct height
   *   Drive straight to the correct distance from the wall
   *   Lower the lift
   *   Back away from the wall
   *   Set the lift to the default height
   */
  public LiftAndRetrieveHatch() {
    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    //addSequential(new FindLine());
    addSequential(new LiftToHeight(Constants.HatchRetrieveDepotHeight, 0, true));//Move the lift to the selected height plus the offset since in the captured position and wait until finished
  //  addSequential(new DriveToPosition(24, Constants.AutoInSpeed, 4));//Move to within 4 inches of the target but a max of 24 inches
   // addSequential(new LiftToHeight(Constants.HatchRetrieveDepotHeight, Constants.HatchDepositDelta, true));//Move the lift down and wait until finished
   // addSequential(new DriveToPosition(-Constants.AutoBackoffDistance, Constants.AutoOutSpeed, 10000));//Back away
   // addSequential(new LiftToHeight(Constants.AutoDefaultLiftHeight, 0,true));//Lift to default height
    addSequential(new WaitButtonsReleased());
 }
}
