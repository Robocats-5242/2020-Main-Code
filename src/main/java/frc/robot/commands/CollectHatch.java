/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
//import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
public class CollectHatch extends CommandGroup {
  /*
   * Pick up hatch from depot
   *   Align with the guide line
   *   Move the lift to the correct height
   *   Drive straight to the correct distance from the wall
   *   Lift the mechanism
   *   Back away from the wall
   *   Set the lift to the default height
   */
  public CollectHatch(double height) {
    addSequential(new HandleCargo(Constants.IntakeStateOff));
    //addSequential(new FindLine());
    addSequential(new LiftToHeight(Constants.HatchRetrieveDepotHeight,0, true));//Lift to opening height
   // addSequential(new DriveToPosition(Constants.AutoStopMaxDistance, Constants.AutoInSpeed, Constants.AutoStopFromDistanceHatch));
   // addSequential(new LiftToHeight(Constants.HatchRetrieveDepotHeight, Constants.HatchGrabDelta,true));//Lift up to grab the hatch
    //addSequential(new DriveToPosition(-Constants.AutoBackoffDistance, Constants.AutoOutSpeed, 10000));//Back away
   // addSequential(new LiftToHeight(Constants.AutoDefaultLiftHeight,0, true));//Move lift to optimal temp height
    addSequential(new WaitButtonsReleased());

  }
}
