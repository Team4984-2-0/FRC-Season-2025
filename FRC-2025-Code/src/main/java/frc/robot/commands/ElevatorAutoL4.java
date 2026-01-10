package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorAutoL4 extends Command {
        private Elevator elevatorSub;
        private boolean Finished;
       
      public ElevatorAutoL4(Elevator elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
            Finished = false;
        }
        @Override
        public void execute(){
            if (elevatorSub.get_encoderElev() > 273 && elevatorSub.get_encoderElev() < 275) {
                Finished = true;
            }
            else {
                if (elevatorSub.get_encoderElev() < 273){
                    if (elevatorSub.get_encoderElev() < 230){
                        elevatorSub.Rotate(-1);
                    }
                    if (elevatorSub.get_encoderElev() > 230){
                        elevatorSub.Rotate(-0.6);
                    }
                }
                if (elevatorSub.get_encoderElev() > 275){
                    if (elevatorSub.get_encoderElev() < 285){
                        elevatorSub.Rotate(0.1);
                    }
                    if (elevatorSub.get_encoderElev() > 285){
                        elevatorSub.Rotate(0.5);
                    }                }
            }
            
        }
        @Override
        public boolean isFinished() {
            return Finished;
        }
        @Override
        public void end(boolean interrupted){
            elevatorSub.RotateStop();
            Finished = false;
        }
}
