// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;




public class Climber extends SubsystemBase {

  public XboxController controller;
  public CANSparkMax leftMotor1, leftMotor2,
                     rightMotor1, rightMotor2;
                     
  public MotorControllerGroup leftGroup, rightGroup;

  public Climber() {
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    leftMotor1 = new CANSparkMax(Constants.CLIMB_MOTOR_1, MotorType.kBrushless);
    leftMotor2 = new CANSparkMax(Constants.CLIMB_MOTOR_2, MotorType.kBrushless);
    rightMotor1 = new CANSparkMax(Constants.CLIMB_MOTOR_3, MotorType.kBrushless);
    rightMotor2 = new CANSparkMax(Constants.CLIMB_MOTOR_4, MotorType.kBrushless);

    leftGroup = new MotorControllerGroup(leftMotor1, leftMotor2);
    rightGroup = new MotorControllerGroup(rightMotor1, rightMotor2);
  }


  public void buttonClimb() {
    if(controller.getYButton()) {
      leftGroup.setInverted(false);
      rightGroup.setInverted(false);
      leftGroup.setVoltage(0.8);
      rightGroup.setVoltage(0.8);

    } else if(controller.getAButton()) {
      leftGroup.setInverted(true);
      rightGroup.setInverted(true);
      leftGroup.setVoltage(0.8);
      rightGroup.setVoltage(0.8);

    } else {
      leftGroup.setVoltage(0);
      rightGroup.setVoltage(0);
    }

    SmartDashboard.putNumber("left group volt", leftMotor1.getBusVoltage());
    SmartDashboard.putNumber("right group volt", rightMotor1.getBusVoltage());
  }

  public void bumperClimb() {
    double power = (controller.getLeftBumper() || controller.getRightBumper()) ? 0.8 : 0;
    boolean invert = controller.getLeftBumper() ? true : false;
    leftGroup.setInverted(invert);
    rightGroup.setInverted(invert);
    leftGroup.setVoltage(power);
    rightGroup.setVoltage(power);
  }

  public void setCoast() {
    if(controller.getXButton()) {
      leftMotor1.setIdleMode(IdleMode.kCoast);
      leftMotor2.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBreak() {
    if(controller.getBButton()) {
      leftMotor1.setIdleMode(IdleMode.kBrake);
      leftMotor2.setIdleMode(IdleMode.kBrake);
      rightMotor1.setIdleMode(IdleMode.kBrake);
      rightMotor2.setIdleMode(IdleMode.kBrake);
    }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
