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
            if (elevatorSub.get_encoderElev() > 145 && elevatorSub.get_encoderElev() < 147) {
                Finished = true;
            }
            else {
                if (elevatorSub.get_encoderElev() >= 147){
                    elevatorSub.Rotate(0.7);
                }
                if (elevatorSub.get_encoderElev() < 145){
                    elevatorSub.Rotate(-0.4);
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
