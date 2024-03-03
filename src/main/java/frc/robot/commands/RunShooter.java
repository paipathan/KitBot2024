// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  public XboxController controller;
  public Shooter shooter;
  
  public RunShooter() {
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    shooter = new Shooter(controller);
    addRequirements(shooter);    
  }

  
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    shooter.shoot();
    shooter.intake();
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
