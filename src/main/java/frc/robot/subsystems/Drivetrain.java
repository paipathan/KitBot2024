// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public WPI_TalonSRX frontLeft, backLeft,
                      frontRight, backRight;
  
  public DifferentialDrive ddrive;
  public XboxController controller;

  public Drivetrain(XboxController controller) {
    this.controller = controller;
    frontLeft = new WPI_TalonSRX(Constants.MOTOR_L1_ID);
    backLeft = new WPI_TalonSRX(Constants.MOTOR_L2_ID);
    frontRight = new WPI_TalonSRX(Constants.MOTOR_R1_ID);
    backRight = new WPI_TalonSRX(Constants.MOTOR_R2_ID);

    frontLeft.follow(backLeft);
    backRight.follow(frontRight);

    ddrive = new DifferentialDrive(frontLeft, frontRight);

  }

  public void move() {
    ddrive.arcadeDrive(-controller.getLeftY(), controller.getRightX());
  }



  @Override
  public void periodic() {
    
  }
}
