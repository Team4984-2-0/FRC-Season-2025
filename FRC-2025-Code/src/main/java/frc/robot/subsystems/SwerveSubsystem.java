package frc.robot.subsystems;

import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UnitConversions;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);


    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI); //AHRS(SPI.Port.kMXP);

    //private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private Field2d myfield;
    private SwerveModulePosition frontleftpos = new SwerveModulePosition (frontLeft.getDrivePosition(),frontLeft.getState().angle);
    private SwerveModulePosition frontrightpos = new SwerveModulePosition (frontRight.getDrivePosition(),frontRight.getState().angle);
    private SwerveModulePosition backLeftpos = new SwerveModulePosition (backLeft.getDrivePosition(),backLeft.getState().angle);
    private SwerveModulePosition backRightpos = new SwerveModulePosition (backRight.getDrivePosition(),backRight.getState().angle);

    private SwerveModulePosition[] WheelPositions = {frontleftpos,frontrightpos,backLeftpos,backRightpos};
    
    

    double xLocationStartFeet = 3.0 / UnitConversions.feetToMeters;  //down field long 
    double yLocationStartFeet = 11.0 / UnitConversions.feetToMeters;  // side to side 
    Rotation2d angleStartDegrees = Rotation2d.fromDegrees(0.0);
    Pose2d startingPosition = new Pose2d(xLocationStartFeet, yLocationStartFeet, angleStartDegrees);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), WheelPositions, startingPosition);

    public SwerveSubsystem() {
        
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders(); 
        myfield = new Field2d();
    }

    public void maxspeed(boolean speed) {
        frontLeft.set_speed(speed);
        frontRight.set_speed(speed);
        backLeft.set_speed(speed);
        backRight.set_speed(speed);
    }
    
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
        

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        //TBD.NeedsARealFixButEliminatesCompileErrors
        SwerveModulePosition[] TBDWheelPositions = new SwerveModulePosition[4];
        odometer.resetPosition(getRotation2d(), TBDWheelPositions, pose);
    }

    @Override
    public void periodic() {
        WheelPositions[0].angle =  frontLeft.getState().angle;
        WheelPositions[0].distanceMeters =  frontLeft.getDrivePosition();

        WheelPositions[1].angle =  frontRight.getState().angle;
        WheelPositions[1].distanceMeters =  frontRight.getDrivePosition();

        WheelPositions[2].angle =  backLeft.getState().angle;
        WheelPositions[2].distanceMeters =  backLeft.getDrivePosition();

        WheelPositions[3].angle =  backRight.getState().angle;
        WheelPositions[3].distanceMeters =  backRight.getDrivePosition();
        

        odometer.update(getRotation2d(), WheelPositions);
        SmartDashboard.putData("Field",myfield);
        myfield.setRobotPose(odometer.getPoseMeters());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("odometer X", getPose().getX());
        SmartDashboard.putNumber("odometer Y", getPose().getY());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //TBD.NeedsARealFixButEliminatesCompileErrors
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}