package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
public class ArmAutoS extends Command {
      private boolean Finished;
        private Launcher intakesub;

        
        public ArmAutoS(Launcher intakesub){
            
            this.intakesub = intakesub;
            addRequirements(intakesub);
            Finished = false;

        }
        
        
         @Override
        public void execute(){
            if (intakesub.get_encoder() > 26 && intakesub.get_encoder() < 28) {
                Finished = true;
            }
            else {
                if (intakesub.get_encoder() > 28){
                    intakesub.Spin(0.3);
                }
                if (intakesub.get_encoder() < 26){
                    intakesub.Spin(-0.3);
                }
             }
            }
            
        
        @Override
        public boolean isFinished() {
            return Finished;
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin();
            Finished = false;
        }
    
    }