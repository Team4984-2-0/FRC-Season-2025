package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {

public float ElevatorHeight = 0;
public float diffrenceHeight = 0;
public float WantHeight = 0;

    private SparkMax motor1;
    private SparkMax motor2;
    

    public Elevator() {
        if (ElevatorHeight > WantHeight) {
           
            diffrenceHeight = ElevatorHeight - WantHeight;
        }
        if (ElevatorHeight < WantHeight) {
            
        }
        if (ElevatorHeight == WantHeight) {
            
        }

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor1 = new SparkMax(9, MotorType.kBrushless);
        motor2 = new SparkMax(10, MotorType.kBrushless);
        motor1.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motor2.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
       
    }

public void Rotate(double value) {
    motor1.set(value);
    motor2.set(-value);
}

public void Rotate() {
    motor1.set(0);
    motor2.set(0);
}


}
