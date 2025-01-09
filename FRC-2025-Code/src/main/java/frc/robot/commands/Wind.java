
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feed;

public class Wind extends Command {

    private Feed FeedSubsystem;

    public Wind(Feed FeedSubsystem) {
        this.FeedSubsystem = FeedSubsystem;
        addRequirements(FeedSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        FeedSubsystem.runMotor(0.50);
    }

    @Override
    public void end(boolean interrupted) {
        FeedSubsystem.runMotor(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}