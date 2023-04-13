// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax upDownMotor;
  CANSparkMax inOutMotor;

  PIDController upDownController = new PIDController(0.1, 0, 0);
  PIDController inOuController = new PIDController(0.1, 0, 0);

  double upDownSetpoint = 0.0;
  double inOutSetpoint = 0.0;
  
  public Arm() {
    upDownMotor = new CANSparkMax(5, MotorType.kBrushless);
    upDownMotor.restoreFactoryDefaults();
    upDownMotor.setSmartCurrentLimit(33);
    upDownMotor.setIdleMode(IdleMode.kBrake);

    inOutMotor = new CANSparkMax(4, MotorType.kBrushless);
    inOutMotor.restoreFactoryDefaults();
    inOutMotor.setSmartCurrentLimit(33);
    inOutMotor.setIdleMode(IdleMode.kBrake);
  }

  public void driveArm(double upDownPower, double inOutPower){
    // upDownMotor.set(upDownPower*0.3);
    // inOutMotor.set(inOutPower*0.4);
  }

  public void changeSetpoints(double newUpDown, double newInOut) {
    System.out.println("Changing Setpoints to " + newUpDown + " and " + newInOut);
    upDownSetpoint = newUpDown;
    inOutSetpoint = newInOut;
  }

  public boolean hasArrived() {
    return Math.abs(upDownMotor.getEncoder().getPosition() - upDownSetpoint) < 10.0 &&
            Math.abs(inOutMotor.getEncoder().getPosition() - inOutSetpoint) < 10.0;
  }

  @Override
  public void periodic() {
    // System.out.println(upDownMotor.getEncoder().getPosition());
    // System.out.println(inOutMotor.getEncoder().getPosition());
    double upDownPower = upDownController.calculate(upDownMotor.getEncoder().getPosition(), upDownSetpoint);
    upDownPower = Math.max(upDownPower, -0.45);
    upDownPower = Math.min(upDownPower, 0.45);
    upDownMotor.set(upDownPower);
    double inOutPower  = inOuController.calculate(inOutMotor.getEncoder().getPosition(), inOutSetpoint);
    inOutPower = Math.max(inOutPower, -0.6);
    inOutPower = Math.min(inOutPower, 0.7);
    inOutMotor.set(inOutPower);
  }
}
