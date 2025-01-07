package frc.robot.commands;




import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feed;


public class Intake extends Command { 
   
    private Launcher launcherSubsystem; 
    private Feed feedsystem;


    public Intake(Launcher launcherSubsystem, Feed feedsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.feedsystem = feedsystem;
        addRequirements(launcherSubsystem);
    }


    @Override
    public void initialize(){


    }


    @Override
    public void execute() {
        launcherSubsystem.runMotor(30.0);
        //feedsystem.runMotor(-20.0);
    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.runMotor(0);
        feedsystem.runMotor(0);

    }

    @Override 
    public boolean isFinished(){
        return false;

    }

    @Override 
    public boolean runsWhenDisabled(){
        return false;
    }
}