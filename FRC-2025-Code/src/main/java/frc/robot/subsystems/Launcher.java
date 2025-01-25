package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Launcher extends SubsystemBase {

    private SparkMax motor3;
  //  private SparkMax motor4;
    

    public Launcher() {
        motor3 = new SparkMax(11, MotorType.kBrushless);
      //  motor4 = new SparkMax(12, MotorType.kBrushless);
       
    }

public void Rotate(double value) {
    motor3.set(value);
   // motor4.set(-value);
}

public void Rotate() {
    motor3.set(0);
//    motor4.set(0);
}


}
