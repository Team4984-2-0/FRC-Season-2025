package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
public class ArmAuto2 extends Command {
      private boolean Finished;
        private Launcher intakesub;

        
        public ArmAuto2(Launcher intakesub){
            
            this.intakesub = intakesub;
            addRequirements(intakesub);
            Finished = false;

        }
        
        
         @Override
        public void execute(){
            if (intakesub.get_encoder() > 3 && intakesub.get_encoder() < 4) {
                Finished = true;
            }
            else {
                if (intakesub.get_encoder() > 4){
                    intakesub.Spin(0.3);
                }
                if (intakesub.get_encoder() < 3){
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