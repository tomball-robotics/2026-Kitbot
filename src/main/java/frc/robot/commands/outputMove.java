package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
public class outputMove extends Command{            
    private Shooter output;


    public outputMove(Shooter set){
        output = set;
    }
    @Override
    public void execute() {
        output.moveOutput(Constants.OutputConstants.motorSpeedMoveOutput);
    }
    @Override
    public void end(boolean interrupted) {
        output.stopOutput();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
