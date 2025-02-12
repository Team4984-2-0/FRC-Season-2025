package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
        private Launcher Launchsub;
        
        public Launch(Launcher Launchsub){
            this.Launchsub = Launchsub;
            addRequirements(Launchsub);
        }
        @Override
        public void execute(){
            Launchsub.Spin(-0.30);
        }
        @Override
        public void end(boolean interrupted){
            Launchsub.Spin(0);
        }
}
