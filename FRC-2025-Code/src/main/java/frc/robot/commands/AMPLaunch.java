
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class AMPLaunch extends Command {

    private Launcher launcherSubsystem;

    public Shooter(Launcher launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        launcherSubsystem.runMotor(-0.40);
    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.runMotor(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}