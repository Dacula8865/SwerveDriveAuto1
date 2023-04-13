// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModuleConstants;

 // results may vary based on robot!

public final class Constants {
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.667;
    public static final double kTurningMotorGearRatio = 1 / 18.0;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kPTurning = 3.5;
  }

  public static final class DriveConstants{

    public static final double kDriveRatio = 6.67;
    public static final double kSteerRatio = 59.166; // 59.166

    public static final double kTrackWidth = Units.inchesToMeters(21);
    // distance between riht and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // distance between front and back wheels 
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), //
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //
    public static final double kPhysicalMaxSpeedMetersPerSecond = 1;

    public static final SwerveModuleConstants kFrontLeftModule = 
      new SwerveModuleConstants(1,
        11, 21,2,-214.14-70,
        6, 7,
        false, false,
        false
    );

    public static final SwerveModuleConstants kFrontRightModule = 
      new SwerveModuleConstants(2,
        14, 24, 3,-268.72 -52,
        2, 3,
        false, false,
        false
    );

    public static final SwerveModuleConstants kBackRightModule = 
      new SwerveModuleConstants(3,
        13, 23,1,-283.44+17,
        0, 1,
        false, false,
        false
    );

    public static final SwerveModuleConstants kBackLeftModule = 
      new SwerveModuleConstants(4,
        12, 22, 0,-319.11+101,
        4, 5,
        false, false,
        false
    );

    public static double kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static double kTeleDriveMaxSpeedMetersPerSecond;
    public static double kTeleDriveMaxAccelerationUnitsPerSecond;
    public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond;
  }
  public class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static double kPXController = 0.5;
    public static double kPYController = 0.5;

    public static double kPThetaController = 0.1/180;

    public static  edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints kThetaControllerConstraints = new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(90, 90);

  }
  public class OIConstants {

    public static final int kDriverRotAxis = 2; //undetermined
    public static double kDeadBand = 0.1; //undetermined
    public static int kDriverXAxis = 0 ; //undetermined
    public static int kDriverYAxis = 1 ; //undetermined
    public static int kDriverFieldOrientedButtonIdx = 1; //undetermined
    public static int kDriverControllerPort = 0; //undetermined
    public static int kOperatorControllerPort = 1;

  }

}
