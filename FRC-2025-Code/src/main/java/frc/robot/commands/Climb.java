package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
        private Climber climbsub;
        
        public Climb(Climber climbsub){
            this.climbsub = climbsub;
            addRequirements(climbsub);
        }
        @Override
        public void execute(){
            climbsub.Spin(0.20);
        }
        @Override
        public void end(boolean interrupted){
            climbsub.Spin(0);
        }
}
