package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ShootAuto extends Command {
        private Arm intakesub;
        public float timer;
        private boolean Finished = false;

        
        public ShootAuto(Arm intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            if (Finished) {
                if(timer >= 10000){
                    Finished = true;
                }
                else{
                    intakesub.Spin(0.10);
                    timer += 0.00001;

                } 
            }
          

        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin(0);
        }
}
