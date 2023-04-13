package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftModule);

    private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightModule);

    private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftModule);

    private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightModule);
    
    private final SwerveModule[] modules = {
        frontLeft, frontRight, backLeft, backRight
    };

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }


    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void turnTo180(){
        
    }


    @Override
    public void periodic(){
     //   odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
    //            backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        for (var mod : modules) {
            mod.periodic();
        }
    //    SmartDashboard.putString("Robot Locatoin", getPose().getTranslation().toString());

    SmartDashboard.putNumber("fL_analog", frontLeft.getAnalogTheta());

    SmartDashboard.putNumber("fR_analog", frontRight.getAnalogTheta());

    SmartDashboard.putNumber("rL_analog", backLeft.getAnalogTheta());

    SmartDashboard.putNumber("rR_analog", backRight.getAnalogTheta());
}

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[1]);
        frontRight.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[3]);
        backRight.setDesiredState(desiredStates[2]);
    }
}
