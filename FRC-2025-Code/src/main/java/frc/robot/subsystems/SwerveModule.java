package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private int power;
    private boolean speed;

    // what does this look like
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        speed = true;
        power = 1;
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveConfig.inverted(driveMotorReversed);

        driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        turnConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        turnConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turnConfig.inverted(turningMotorReversed);

        turningMotor.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        //SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel(), getAbsoluteEncoderRad());
        resetEncoders();
     //   SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " start", getAbsoluteEncoderRad());
        
    }
    public void set_speed(int howfast) {
        power = howfast;
    }
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        if (absoluteEncoder.getChannel() == 0) {
        //System.out.println("Swerve[" + absoluteEncoder.getChannel() + "] voltage " + absoluteEncoder.getVoltage());
        //System.out.println("Swerve[" + absoluteEncoder.getChannel() + "] rio5v " + RobotController.getVoltage5V());
        }
        double angle = absoluteEncoder.getVoltage() / (RobotController.getVoltage5V());
        
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderRadtest() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        return angle;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " absolute before reset", getAbsoluteEncoderRad());
        turningEncoder.setPosition(getAbsoluteEncoderRad());
        //SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " turning encoder", turningEncoder.getPosition());

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // This needs to be changed since it is depricated
        state = SwerveModuleState.optimize(state, getState().angle);
        switch (power) {
            case 1:
                driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/1.3);
                break;
            case 2:
                driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/2);
                break;
            case 3: 
                driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
                break;
            default:
                driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/2);
                break;
        }
/*         if (speed) {
            driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/1.3);
        }
        else {
            driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/2);
        } */
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
      //  SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
       // SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " live", getAbsoluteEncoderRad());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}