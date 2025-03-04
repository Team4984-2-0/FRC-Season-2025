package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climbbutback extends Command {
        private Climber climbsub;
        
        public Climbbutback(Climber climbsub){
            this.climbsub = climbsub;
            addRequirements(climbsub);
        }
        @Override
        public void execute(){
            climbsub.Spin(-0.45);
        }
        @Override
        public void end(boolean interrupted){
            climbsub.Spin(0);
        }
}
