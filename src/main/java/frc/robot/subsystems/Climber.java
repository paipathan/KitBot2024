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
  public CANSparkMax motor1, motor2,
                     motor3, motor4;

  public MotorControllerGroup leftMotors, rightMotors;

  public Climber() {
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    motor1 = new CANSparkMax(Constants.CLIMB_MOTOR_1, MotorType.kBrushless);
    // motor2 = new CANSparkMax(Constants.CLIMB_MOTOR_2, MotorType.kBrushless);
    // motor3 = new CANSparkMax(Constants.CLIMB_MOTOR_3, MotorType.kBrushless);
    // motor4 = new CANSparkMax(Constants.CLIMB_MOTOR_4, MotorType.kBrushless);
    leftMotors = new MotorControllerGroup(motor1, motor2);
    // rightMotors = new MotorControllerGroup(motor3, motor4);
  }


  public void buttonClimb() {
    if(controller.getYButton()) {
      motor1.setVoltage(0.4);
      motor2.setVoltage(0.4);

    } else if(controller.getAButton()) {
      motor1.setVoltage(-0.4);
      motor2.setVoltage(-0.4);

    } else {
      motor1.setVoltage(0);
      motor2.setVoltage(0);
    }

    SmartDashboard.putNumber("m1 volt", motor1.getBusVoltage());
    SmartDashboard.putNumber("m2 volt", motor2.getBusVoltage());
    // SmartDashboard.putNumber("m3 volt", motor3.getBusVoltage());
    // SmartDashboard.putNumber("m4 volt", motor4.getBusVoltage());
  }

  public void joystickClimb() {

    // if(MathUtil.applyDeadband(controller.getLeftY(), 0.1) != 0) {
    //   motor1.set(controller.getLeftY());
    //   motor2.set(controller.getLeftY());
    //   motor3.set(controller.getLeftY());
    //   motor4.set(controller.getLeftY());

    //   // motor1.set(MathUtil.applyDeadband(controller.getLeftY(), 0.1));
    //   // motor2.set(MathUtil.applyDeadband(controller.getLeftY(), 0.1));
    //   // motor3.set(MathUtil.applyDeadband(controller.getLeftY(), 0.1));
    //   // motor4.set(MathUtil.applyDeadband(controller.getLeftY(), 0.1));
    // } else {
    //   motor1.set(0.8);
    //   motor2.set(0.8);
    //   motor3.set(0.8);
    //   motor4.set(0.8);
    // }

    motor1.set(-controller.getLeftY());
    motor2.set(-controller.getLeftY());
    // motor3.set(-controller.getLeftY());
    // motor4.set(-controller.getLeftY());


    SmartDashboard.putNumber("m1 volt", motor1.getBusVoltage());
    SmartDashboard.putNumber("m2 volt", motor2.getBusVoltage());
    // SmartDashboard.putNumber("m3 volt", motor3.getBusVoltage());
    // SmartDashboard.putNumber("m4 volt", motor4.getBusVoltage());
  }

  public void triggerClimb() {}


  public void bumperClimb() {
    double power = (controller.getLeftBumper() || controller.getRightBumper()) ? 0.8 : 0;
    boolean invert = controller.getLeftBumper() ? true : false;
    leftMotors.setInverted(invert);
    // rightMotors.setInverted(invert);
    leftMotors.setVoltage(power);
    // rightMotors.setVoltage(power);
  }

  public void setCoast() {
    if(controller.getXButton()) {
      motor1.setIdleMode(IdleMode.kCoast);
      motor2.setIdleMode(IdleMode.kCoast);
      // motor3.setIdleMode(IdleMode.kCoast);
      // motor4.setIdleMode(IdleMode.kCoast);

    }
  }

  public void setBreak() {
    if(controller.getBButton()) {
      motor1.setIdleMode(IdleMode.kBrake);
      motor2.setIdleMode(IdleMode.kBrake);
      // motor3.setIdleMode(IdleMode.kBrake);
      // motor4.setIdleMode(IdleMode.kBrake);
    }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
