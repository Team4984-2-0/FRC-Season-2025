package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorAutoPickup extends Command {
        private Elevator elevatorSub;
        private boolean Finished;
       
      public ElevatorAutoPickup(Elevator elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
            Finished = false;
        }
        @Override
        public void execute(){
            if (elevatorSub.get_encoderElev() >= 153 && elevatorSub.get_encoderElev() <= 154) {
                Finished = true;
            }
            else {
                if (elevatorSub.get_encoderElev() >= 154){
                    
                    elevatorSub.Rotate(0.9);
                }
                if (elevatorSub.get_encoderElev() <= 153){
                    elevatorSub.Rotate(-0.9);
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
