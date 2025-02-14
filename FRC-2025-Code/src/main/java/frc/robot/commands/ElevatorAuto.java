package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoFeeder;

public class ElevatorAuto extends Command {
        private AutoFeeder elevatorSub;
        public ElevatorAuto(AutoFeeder elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
        }
        @Override
        public void execute(){
            while (elevatorSub.get_encoder() < 25) {
                elevatorSub.pickupAuto(-0.5);
            }
        }
        @Override
        public void end(boolean interrupted){
            elevatorSub.Rotate(0);
        }
}
