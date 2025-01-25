package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorDown extends SubsystemBase {

    private SparkMax motor1;
    private SparkMax motor2;
    

    public ElevatorDown() {
        motor1 = new SparkMax(9, MotorType.kBrushless);
        motor2 = new SparkMax(10, MotorType.kBrushless);
       
    }

public void Rotate(double value) {
    motor1.set(-value);
    motor2.set(value);
}

public void Rotate() {
    motor1.set(0);
    motor2.set(0);
}


}
