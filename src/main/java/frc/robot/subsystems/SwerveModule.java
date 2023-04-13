package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule {

    private final SwerveModuleConstants modConstants;

    private final TalonFX driveMotor;
    private final TalonSRX turningMotor;
    
    // private final CANEncoder driveEncoder;
    // private final CANEncoder turningEncoder;

    private final PIDController turningPidController;

    private final String name;

  //  private final Encoder pgEncoder;
    AnalogPotentiometer encoder;

    private Rotation2d lastAngle = new Rotation2d();
    
    public SwerveModule(SwerveModuleConstants module) {
        modConstants = module;
        //pgEncoder = new Encoder(module.angleEncA, module.angleEncB);
        encoder = new AnalogPotentiometer(module.angleEncAnalog,360,module.angEncAnalog_offset);
        

        driveMotor = new TalonFX(module.driveId);
        turningMotor = new TalonSRX(module.turnId);

        driveMotor.setInverted(module.driveInvert);
        turningMotor.setInverted(module.turnInvert);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // driveMotor.configSelectedFeedbackCoefficient();
        // driveEncoder = driveMotor.getEncoder();
        // turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
       // pgEncoder.setDistancePerPulse((1/ (Constants.DriveConstants.kSteerRatio * 7)) * 360);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        name = "Mod" + modConstants.moduleNumber;        
        resetEncoders();
    }

    public double getTurningPosition(){
        //return pgEncoder.getDistance();
       return Math.IEEEremainder(encoder.get(),360);
    }

    public double getAnalogTheta(){
        return encoder.get();
    }

    public double getDriveVelocity(){
        var motorRps = driveMotor.getSelectedSensorVelocity() / 2048 * 10;
        var wheelRps = motorRps / kDriveMotorGearRatio;
        var wheelMps = wheelRps * kWheelCircumferenceMeters;
        return wheelMps;
    }

    public double getTurningVelociy(){
        return 0;
        // return turningEncoder.getVelocity();
    }

    public void resetEncoders(){
       // pgEncoder.reset();
        driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.angle = lastAngle;
        }

        // state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(Units.degreesToRadians(getTurningPosition()), state.angle.getRadians()));
        // turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(Units.degreesToRadians(getTurningPosition()), Rotation2d.fromDegrees(45).getRadians()));
        SmartDashboard.putString("Swerve[" + modConstants.moduleNumber + "] state", state.toString());
        lastAngle = state.angle;
    }

    public void periodic() {
        SmartDashboard.putNumber(name + "-PgEnc", encoder.get());
        SmartDashboard.putNumber(name + "-DrSp", getDriveVelocity());
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }

    
}
