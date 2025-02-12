package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Intake extends Command {
        private Launcher intakesub;
        
        public Intake(Launcher intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.Spin(0.30);
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin(0);
        }
}
