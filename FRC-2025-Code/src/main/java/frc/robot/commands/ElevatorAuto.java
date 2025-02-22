package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoFeeder;

public class ElevatorAuto extends Command {
        private AutoFeeder elevatorSub;
        private boolean Finished;
       
      public ElevatorAuto(AutoFeeder elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
            Finished = false;
        }
        @Override
        public void execute(){
            if (elevatorSub.get_encoder() > 24 && elevatorSub.get_encoder() < 21) {
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
