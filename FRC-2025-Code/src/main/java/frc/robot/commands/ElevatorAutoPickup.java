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
            if (elevatorSub.get_encoder() > 24 && elevatorSub.get_encoder() < 28) {
                Finished = true;
            }
            else {
                if (elevatorSub.get_encoder() >= 28){
                    elevatorSub.Rotate(0.6);
                }
                if (elevatorSub.get_encoder() < 24){
                    elevatorSub.Rotate(-0.6);
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
