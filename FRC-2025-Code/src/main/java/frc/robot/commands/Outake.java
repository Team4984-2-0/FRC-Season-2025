package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Outake extends Command {
        private Launcher intakesub;
        
        public Outake(Launcher intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.Spin(20);
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin(0);
        }
}
