package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AutoFeeder extends SubsystemBase {


private final RelativeEncoder motor1Encoder;
    private final RelativeEncoder motor2Encoder;

    private SparkMax motor1;
    private SparkMax motor2;
    

    public AutoFeeder() {


        
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor1 = new SparkMax(9, MotorType.kBrushless);
        motor2 = new SparkMax(10, MotorType.kBrushless);
        motor1Encoder = motor1.getEncoder();
        motor2Encoder = motor2.getEncoder();
        motor1.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motor2.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
       
    }
    // Commenting out this for now, working on elevator auto move up / down. (2/12/25)

public void Rotate(double value) {
    SmartDashboard.putNumber("encoder elevator motor1", motor1Encoder.getPosition());
    SmartDashboard.putNumber("encoder elevator motor2", motor2Encoder.getPosition());
    motor1.set(value);
    motor2.set(-value);
}

public void Rotate() {
    SmartDashboard.putNumber("encoder elevator motor1", motor1Encoder.getPosition());
    SmartDashboard.putNumber("encoder elevator motor2", motor2Encoder.getPosition());
    motor1.set(0);
    motor2.set(0);
}
public double get_encoder(){
    return motor2Encoder.getPosition();
}

public void pickupAuto(double value){
    if(motor2Encoder.getPosition() < 27.5) {
        SmartDashboard.putNumber("encoder elevator motor1", motor1Encoder.getPosition());
        SmartDashboard.putNumber("encoder elevator motor2", motor2Encoder.getPosition());
        motor1.set(value);
        motor2.set(-value);  
    }
    else{
        motor1.set(0);
        motor2.set(0);
    }

}
}
