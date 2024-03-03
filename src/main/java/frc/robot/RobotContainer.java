// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RunClimber;
import frc.robot.commands.joyDrive;

public class RobotContainer {



  //---------- COMMANDS -----------//

  public joyDrive joyDrive; 
  public RunClimber runClimber;

  //-------------------------------//

  public RobotContainer() { // initialize commands
    joyDrive = new joyDrive();
    runClimber = new RunClimber();
  }

  public Command getAutoCommand() {
    return new PathPlannerAuto("straightauto");
  }
  
  public Command[] getTeleCommands() { 
    Command[] commands = new Command[] {joyDrive, runClimber};
    return commands;
  }
}