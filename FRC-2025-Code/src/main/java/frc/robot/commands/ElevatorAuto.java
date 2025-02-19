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
            elevatorSub.Rotate(-0.6);
            if (elevatorSub.get_encoder() >= 27) {
                Finished = true;
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
