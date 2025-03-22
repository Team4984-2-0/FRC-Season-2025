package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class ElevatorGoUp extends Command {
        private Elevator elevatorSub;
        public ElevatorGoUp(Elevator elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
        }
        @Override
        public void execute(){
              elevatorSub.Rotate(0.6);
        }
        @Override
        public void end(boolean interrupted){
            elevatorSub.Rotate(0);
        }
}
