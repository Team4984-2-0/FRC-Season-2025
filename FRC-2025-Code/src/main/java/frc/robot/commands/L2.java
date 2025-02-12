/*package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class L2 extends Command {
        private Elevator elevatorSub;

        public L2(Elevator elevatorSub){
            this.elevatorSub = elevatorSub;
            addRequirements(elevatorSub);
        }
        @Override
        public void execute(){
            elevatorSub.WantHeight = 67;
        }
        @Override
        public void end(boolean interrupted){
            elevatorSub.Rotate(0);
        }
}
*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class L2 extends Command {
        private Launcher intakesub;
        
        public L2(Launcher intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.Spin(-30);
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin(0);
        }
}
