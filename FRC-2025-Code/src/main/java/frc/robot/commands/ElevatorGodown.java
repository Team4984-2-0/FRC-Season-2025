package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorDown;

public class ElevatorGodown extends Command {
        private ElevatorDown elevatorSub;
        public ElevatorGodown(ElevatorDown elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
        }
        @Override
        public void execute(){
              elevatorSub.Rotate(5);
        }
        @Override
        public void end(boolean interrupted){
            elevatorSub.Rotate(0);
        }
}
