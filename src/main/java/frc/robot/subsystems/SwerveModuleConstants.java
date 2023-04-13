package frc.robot.subsystems;

public class SwerveModuleConstants {
    public final int moduleNumber;
    public final int driveId;
    public final int turnId;
    public final int angleEncA;
    public final int angleEncB;
    public final int angleEncAnalog;
    public final boolean driveInvert;
    public final boolean turnInvert;
    public final boolean encoderInvert;
    public final double angEncAnalog_offset;

    public SwerveModuleConstants(
        int moduleNumber,
        int driveId, int turnId,int angleEncAnalog, double angEncAnalog_offset,
        int angleEncA, int angleEncB,
        boolean driveInvert, boolean turnInvert,
        boolean encoderInvert
    ) {
        this.moduleNumber = moduleNumber;
        this.driveId = driveId;
        this.turnId = turnId;
        this.angleEncA = angleEncA;
        this.angleEncB = angleEncB;
        this.driveInvert = driveInvert;
        this.turnInvert = turnInvert;
        this.encoderInvert = encoderInvert;
        this.angleEncAnalog = angleEncAnalog;
        this.angEncAnalog_offset = angEncAnalog_offset;
    }
}
