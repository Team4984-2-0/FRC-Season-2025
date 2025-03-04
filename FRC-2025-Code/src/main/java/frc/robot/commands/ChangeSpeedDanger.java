package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ChangeSpeedDanger extends Command {
        private final SwerveSubsystem swerveSubsystem;
        
        public ChangeSpeedDanger(SwerveSubsystem swerveSubsystem){
            this.swerveSubsystem = swerveSubsystem;
            addRequirements(swerveSubsystem);
        }
        @Override
        public void execute(){
            swerveSubsystem.maxspeed(3);
        }
        @Override
        public void end(boolean interrupted){
            
        }
}

