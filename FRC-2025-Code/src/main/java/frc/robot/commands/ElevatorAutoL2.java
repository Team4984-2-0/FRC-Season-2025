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
            if (elevatorSub.get_encoder() > 0 && elevatorSub.get_encoder() < 3) {
                Finished = true;
            }
            else {
                if (elevatorSub.get_encoder() >= 3){
                    elevatorSub.Rotate(0.3);
                }
                if (elevatorSub.get_encoder() < 0){
                    elevatorSub.Rotate(-0.3);
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
