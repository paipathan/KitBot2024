// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


public class joyDrive extends Command {

  public Drivetrain drive;
  public XboxController controller;

  public joyDrive() {
    this.controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    this.drive = new Drivetrain(controller);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    drive.move();
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
