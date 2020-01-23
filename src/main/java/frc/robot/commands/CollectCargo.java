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
import frc.robot.Robot;

public class CollectCargo extends CommandGroup {
  private String CommandName = "CollectCargo";
  /*
   * Pick up a cargo ball from either the ground or the depot
   * When picking up from the ground the following sequence is performed
   *   Move lift to ground pickup height
   *   Turn on the intake, don't wait for it to capture a ball though
   * When picking up from the depot the following sequence is performed
   *   Align with the guide line
   *   Move the lift to the correct height
   *   Drive straight to the correct distance from the wall
   *   Turn on the intake and wait for a ball to be captured
   *   Back away from the wall
   *   Set the lift to the default height
   */
  public CollectCargo(int location) {
    Robot.logMessage(CommandName, "constructor");
    if(location == Constants.CargoRetrieveLocationGround){
      addSequential(new HandleCargo(Constants.IntakeStateIn));
      //addSequential(new FindLine());//Ground doesn't need the line finder
      addSequential(new LiftToHeight(Constants.CargoRetrieveGroundHeight,0, false));//Lift to height, no block, no offset
//      addSequential(new HandleCargo(Constants.IntakeStateIn));
      addSequential(new WaitButtonsReleased());
    }
    else if(location == Constants.CargoRetrieveLocationDepot){
      //addSequential(new FindLine());
      addSequential(new HandleCargo(Constants.IntakeStateIn));
      addSequential(new LiftToHeight(Constants.CargoRetrieveDepotHeight,0, true));//Lift to height, block, no offset
    //  addSequential(new DriveToPosition(Constants.AutoStopMaxDistance, Constants.AutoInSpeed, Constants.AutoStopFromDistanceCargo));
  //    addSequential(new HandleCargo(Constants.IntakeStateIn));
    //  addSequential(new DriveToPosition(-Constants.AutoBackoffDistance, Constants.AutoOutSpeed, 10000));
     // addSequential(new LiftToHeight(Constants.AutoDefaultLiftHeight,0, false));//When done move to optimal height but don't block
      addSequential(new WaitButtonsReleased());
    }

  }
}
