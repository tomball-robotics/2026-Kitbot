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
public class Higher extends Command{
    private Shooter input;
    public Higher(Shooter set){
        input = set;
    }
    public void execute() {
      //remove before first comp
    if(RobotController.getBatteryVoltage() < 10.5){
      input.ReV(.9);
    }
      else{
        input.ReV(.85);
  }
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


