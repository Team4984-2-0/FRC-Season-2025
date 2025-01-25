package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
        private Launcher launchSub;
        
        public Launch(Launcher elevatorSub){
            this.launchSub = launchSub;
            addRequirements(launchSub);
        }
        @Override
        public void execute(){
            launchSub.Rotate(30);
        }
        @Override
        public void end(boolean interrupted){
            launchSub.Rotate(0);
        }
}
