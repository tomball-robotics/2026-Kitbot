package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
public class inputMove extends Command{
    private Shooter input;
    public inputMove(Shooter set){
        input = set;
    }
    @Override
    public void execute() {
        input.moveInput(Constants.InputConstants.motorSpeedMoveInput);
      }
    @Override
    public void end(boolean interrupted) {
        input.stopOutput();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
}
