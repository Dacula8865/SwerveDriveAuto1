// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MB_Math;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToAngle extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  double setpoint;
  double kP;
  double maxSpeed;
  /** Creates a new TurnTo180. */
  public TurnToAngle(SwerveSubsystem swerveSubsystem, double setpoint, double kP, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.setpoint = setpoint;
    this.kP = kP;
    this.maxSpeed = maxSpeed;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = swerveSubsystem.getHeading();
    double pTerm = MB_Math.angDiffDeg(setpoint, measurement) * kP;

    pTerm = Math.max(pTerm,-maxSpeed);
    pTerm = Math.min(pTerm,maxSpeed);

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(0,0, -pTerm);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
