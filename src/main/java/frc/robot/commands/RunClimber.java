// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RunClimber extends Command {


  public XboxController controller;
  public Climber climber;

  
  public RunClimber() {
    this.controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    this.climber = new Climber(controller);
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.buttonClimb();
    addRequirements(climber);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
