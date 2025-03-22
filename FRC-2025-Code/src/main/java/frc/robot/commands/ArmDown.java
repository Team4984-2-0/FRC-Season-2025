package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDown extends Command {
        private Arm intakesub;
        
        public ArmDown(Arm intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.Spin(-0.20);
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin(0);
        }
}
