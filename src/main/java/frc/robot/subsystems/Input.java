package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Input extends SubsystemBase{
     private  SparkMax inputMotor;
    private SparkMaxConfig motorConfig;
     public Input(){
     inputMotor = new SparkMax(Constants.InputConstants.inputMotor, MotorType.kBrushed);
        motorConfig = new SparkMaxConfig();

       motorConfig.smartCurrentLimit(150)
       .idleMode(IdleMode.kCoast)
       .inverted(false);
     }
     public void moveInput(double speed){
      System.out.println("o");
     inputMotor.set(speed);
     }
     public void stopInput(){
        inputMotor.stopMotor();
}
}
