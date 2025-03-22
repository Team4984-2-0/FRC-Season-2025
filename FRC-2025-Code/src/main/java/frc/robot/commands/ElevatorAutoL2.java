package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorAutoL2 extends Command {
        private Elevator elevatorSub;
        private boolean Finished;
       
      public ElevatorAutoL2(Elevator elevatorSub){
            this.elevatorSub = elevatorSub;
                
            addRequirements(elevatorSub);
            Finished = false;
        }
        @Override
        public void execute(){
            if (elevatorSub.get_encoderElev() > 22 && elevatorSub.get_encoderElev() < 25) {
                Finished = true;
            }
            else {
                if (elevatorSub.get_encoderElev() >= 25){
                    elevatorSub.Rotate(0.7);
                }
                if (elevatorSub.get_encoderElev() < 22){
                    elevatorSub.Rotate(-0.7);
                }
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
