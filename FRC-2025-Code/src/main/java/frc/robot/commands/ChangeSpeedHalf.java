package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;

public class ChangeSpeedHalf extends Command {
        private final SwerveSubsystem swerveSubsystem;
        
        public ChangeSpeedHalf(SwerveSubsystem swerveSubsystem){
            this.swerveSubsystem = swerveSubsystem;
            addRequirements(swerveSubsystem);
        }
        @Override
        public void execute(){
            swerveSubsystem.maxspeed(false);
        }
        @Override
        public void end(boolean interrupted){
            
        }
}

