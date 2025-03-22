package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Launchfast extends Command {
        private Arm Launchsub;
        
        public Launchfast(Arm Launchsub){
            this.Launchsub = Launchsub;
            addRequirements(Launchsub);
        }
        @Override
        public void execute(){
            Launchsub.Spin(0.30);
        }
        @Override
        public void end(boolean interrupted){
            Launchsub.Spin(0);
        }
}
