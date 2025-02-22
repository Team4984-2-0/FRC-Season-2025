package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ChangeSpeedMax extends Command {
        private final SwerveSubsystem swerveSubsystem;
        
        public ChangeSpeedMax(SwerveSubsystem swerveSubsystem){
            this.swerveSubsystem = swerveSubsystem;
            addRequirements(swerveSubsystem);
        }
        @Override
        public void execute(){
            swerveSubsystem.maxspeed(true);
        }
        @Override
        public void end(boolean interrupted){
            
        }
}

