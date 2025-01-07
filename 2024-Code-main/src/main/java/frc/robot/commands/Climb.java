package frc.robot.commands;




import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Climber;



public class Climb extends Command {
    
    Climber climberSubsystem;



    public Climb(Climber climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }


    @Override
    public void initialize(){


    }


    @Override
    public void execute() {
        climberSubsystem.runMotor(30.0);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.runMotor(0);
    
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