package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorGo extends Command {
        private Elevator elevatorSub;
        public int elevatorSpeed = 5;
        public ElevatorGo(Elevator elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
        }
        @Override
        public void execute(){
            elevatorSpeed += 5;
            elevatorSub.Rotate(elevatorSpeed);
        }
        @Override
        public void end(boolean interrupted){
            elevatorSub.Rotate(0);
        }
}
