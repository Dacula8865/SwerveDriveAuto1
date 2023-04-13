// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveInDirectionTimed;
import frc.robot.commands.SleepCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EF;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Arm armSubsystem = new Arm();
  private final EF efSubsystem = new EF();

  private Timer timer = new Timer();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

  public RunCommand driveArm = new RunCommand(()->armSubsystem.driveArm(operatorJoystick.getY(),operatorJoystick.getTwist()),armSubsystem);

  public boolean commanded_lift = false;
  // public boolean commanded_score = false;
  // public boolean commanded_idle = false;
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -driverJoystick.getY() / 1.5,
            () -> -driverJoystick.getX() / 1.5,
            () -> -driverJoystick.getTwist() / 1.5,
            () -> !driverJoystick.getTriggerPressed()));

            armSubsystem.setDefaultCommand(driveArm);
        //    efSubsystem.setDefaultCommand(new RunCommand(efSubsystem::stopped,efSubsystem));


           autoChooser.addOption("roll backwards", new DriveInDirectionTimed(swerveSubsystem,-0.25,0,0,2.5));
            autoChooser.addOption("place, roll back", placeAndRollBackAuto());

           autoChooser.setDefaultOption("do nothing", null);
           SmartDashboard.putData("auto chooser",autoChooser);

    configureButtonBindings();
  }

  Command placeAndRollBackAuto(){
Command raiseArm1 = new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-0.15),armSubsystem);
Command sleepCommand = new SleepCommand(1.5);
Command raiseArm2 = new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-46.6),armSubsystem);
Command rollForward = new DriveInDirectionTimed(swerveSubsystem,0.10,0,0,2.5);
Command eject = new InstantCommand(()->efSubsystem.outtake());
Command sleep1 = new SleepCommand(1.5);
Command retractArm1 = new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-0.15),armSubsystem);
Command rollBackwards = new DriveInDirectionTimed(swerveSubsystem, -0.25, 0, 0, .5);
Command retractArm2 = new InstantCommand(()->armSubsystem.changeSetpoints(.0,-0.15),armSubsystem);
Command rollBackwards2 = new DriveInDirectionTimed(swerveSubsystem, -0.25, 0, 0, .5);
Command stopEF = new InstantCommand(efSubsystem::stopped,efSubsystem);

return raiseArm1.andThen(sleepCommand.andThen(raiseArm2.andThen(rollForward.andThen(eject.andThen(sleep1.andThen(retractArm1.andThen(rollBackwards.andThen(retractArm2.andThen(rollBackwards2.andThen(stopEF))))))))));
//return raiseArm1.andThen(sleepCommand.andThen(raiseArm2.andThen(rollForward.andThen(eject.andThen(sleep1).andThen(rollBackwards.andThen(retractArm1.andThen(rollBackwards2.andThen(retractArm2))))))));
  }

  public void init(){
  }
  public void periodic() {
    armSubsystem.periodic();
  }

  private void configureButtonBindings() {
     new JoystickButton(driverJoystick, 8).whenPressed(() -> swerveSubsystem.zeroHeading());
    new JoystickButton(operatorJoystick, 5).onTrue(new InstantCommand(efSubsystem::outtake,efSubsystem));

    new JoystickButton(operatorJoystick, 3).onTrue(new InstantCommand(efSubsystem::intake,efSubsystem));

    new JoystickButton(operatorJoystick, 5).onFalse(new InstantCommand(efSubsystem::stopped,efSubsystem));

    new JoystickButton(operatorJoystick, 3).onFalse(new InstantCommand(efSubsystem::stopped,efSubsystem));

    new JoystickButton(operatorJoystick, 11).onTrue(new InstantCommand(()->armSubsystem.changeSetpoints(.0,-0.15),armSubsystem));

    new JoystickButton(operatorJoystick, 12).onTrue(new InstantCommand(()->armSubsystem.changeSetpoints(4.0,-3.0),armSubsystem));

    new JoystickButton(operatorJoystick, 10).onTrue(new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-0.15),armSubsystem));

    new JoystickButton(operatorJoystick, 8).onTrue(new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-18),armSubsystem));

    new JoystickButton(operatorJoystick, 9).onTrue(new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-16.0),armSubsystem));

    new JoystickButton(operatorJoystick, 7).onTrue(new InstantCommand(()->armSubsystem.changeSetpoints(56.5,-46.6),armSubsystem));

    new JoystickButton(driverJoystick, 9).whileTrue(new TurnToAngle(swerveSubsystem,180,1/25.0,0.6));

  }

  public void run_auto() {
    if (!commanded_lift) {
      armSubsystem.changeSetpoints(60, -70);
      commanded_lift = true;
    }
    if (armSubsystem.hasArrived()) {
      efSubsystem.outtake();
      armSubsystem.changeSetpoints(0.0, 0.0);
    }
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
//    //1.Create trajectory settings
//    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//            AutoConstants.kMaxSpeedMetersPerSecond,
//            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                    .setKinematics(DriveConstants.kDriveKinematics);
//
//    //2. Generate trajectory
//    var trajectoryOne =
//    TrajectoryGenerator.generateTrajectory(
//      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
//      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
//      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
//
//    var trajectoryTwo =
//    TrajectoryGenerator.generateTrajectory(
//      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
//      List.of(new Translation2d(4, 4), new Translation2d(6, 3)),
//      new Pose2d(6, 0, Rotation2d.fromDegrees(0)),
//      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
//
//    Trajectory concatTraj = trajectoryOne.concatenate(trajectoryTwo);
//
//    //3. Define PID controllers for tracking trajectory
//    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
//    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
//    ProfiledPIDController thetaController = new ProfiledPIDController(
//            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//    thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//    //4. Construct command to follow trajectory
//    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//            concatTraj,
//            swerveSubsystem::getPose,
//            DriveConstants.kDriveKinematics,
//            xController,
//            yController,
//            thetaController,
//            swerveSubsystem::setModuleStates,
//            swerveSubsystem);
//
//    //5. Add some init and wrap-up, and return everything
//    return new SequentialCommandGroup(
//            new InstantCommand(() -> swerveSubsystem.resetOdometry(concatTraj.getInitialPose())),
//            swerveControllerCommand,
//            new InstantCommand(() -> swerveSubsystem.stopModules()));
//    }


  }

}