// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EF extends SubsystemBase {
  /** Creates a new EF. */
  TalonSRX efMotor;
  public EF() {
efMotor = new TalonSRX(3);
efMotor.configFactoryDefault();
efMotor.configPeakCurrentLimit(25);
efMotor.configPeakCurrentDuration(25);
efMotor.configContinuousCurrentLimit(20);
efMotor.enableCurrentLimit(true);
  }

  public void outtake(){
    efMotor.set(ControlMode.PercentOutput, 1);
    
  }
  
  public void intake(){
    efMotor.set(ControlMode.PercentOutput, -1);
    
  }
  public void stopped(){
    efMotor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
