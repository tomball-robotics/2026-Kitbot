package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
public class Reverse extends Command{
    private Shooter input;
    public Reverse(Shooter set){
        input = set;
    }
    public void execute() {
      //remove before first comp
    
      input.ReV(-1);
    
      
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