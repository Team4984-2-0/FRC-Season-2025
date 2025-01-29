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



    // what does this look like
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
     //   driveMotor.setIdleuMode(IdleMode.kCoast);
     //   driveMotor.setInverted(driveMotorReversed);
     //   turningMotor.setInverted(turningMotorReversed);
     //   turningMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

//        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
//        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
//        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
//        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        ///Keith
        SparkMaxConfig config_ = new SparkMaxConfig();
        config_.idleMode(SparkBaseConfig.IdleMode.kCoast);
        ////driveEncoder
        //config_.absoluteEncoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //config_.absoluteEncoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        config_.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        config_.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        config_.inverted(absoluteEncoderReversed);
        driveMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
        config_.idleMode(SparkBaseConfig.IdleMode.kBrake);
        ////turningEncoder
        config_.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        config_.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        //config_.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //config_.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        //config_.absoluteEncoder.inverted(absoluteEncoderReversed);
        turningMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        //SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel(), getAbsoluteEncoderRad());
        resetEncoders();
        SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " start", getAbsoluteEncoderRad());
        
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

/*          if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }  */
        state = SwerveModuleState.optimize(state, getState().angle);
        if (absoluteEncoder.getChannel() == 0) {
        //System.out.println(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        }
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " live", getAbsoluteEncoderRad());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
