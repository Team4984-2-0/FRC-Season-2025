package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Launcher extends SubsystemBase {

    private SparkMax motor3;
    private SparkMax motor4;
     

    public Launcher() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor3 = new SparkMax(11, MotorType.kBrushless);
        motor4 = new SparkMax(12, MotorType.kBrushless);
       
        motor3.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
       
       
    }

public void Spin(double value) {
    motor3.set(value);
    motor4.set(-value);
}



public void Spin() {
    motor3.set(0);
    motor4.set(0);
   
}


}
