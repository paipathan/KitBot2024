// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public XboxController controller;
  public WPI_TalonSRX top, bottom;


  public Shooter(XboxController controller) {
    this.controller = controller;
    top = new WPI_TalonSRX(Constants.SHOOTER_1_ID);
    bottom = new WPI_TalonSRX(Constants.SHOOTER_2_ID);
  }

  public void shoot() {
    double topPower = controller.getRightBumper() ? 1 : 0;
    double bottomPower = controller.getRightTriggerAxis();
    top.set(topPower);
    bottom.set(bottomPower);
  }

  public void intake() {
    top.set(controller.getLeftTriggerAxis());
    bottom.set(controller.getLeftTriggerAxis());
  }

  @Override
  public void periodic() {}
}
