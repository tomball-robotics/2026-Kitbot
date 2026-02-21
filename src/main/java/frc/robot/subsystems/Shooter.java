package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    private  SparkMax outputMotor;
    private  SparkMax inputMotor;
    private SparkMaxConfig motorConfig;
    
     public Shooter(){
     outputMotor = new SparkMax(Constants.OutputConstants.outputMotor, MotorType.kBrushed);
     inputMotor = new SparkMax(Constants.InputConstants.inputMotor, MotorType.kBrushed);
     motorConfig = new SparkMaxConfig();
       motorConfig.smartCurrentLimit(40)
       .idleMode(IdleMode.kCoast)
       .inverted(false);
       outputMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       inputMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        }
        public void ReV(double speed){
                outputMotor.set(-speed);
                     }
     public void moveOutput(double speed){
        outputMotor.set(-speed);
        inputMotor.set(speed);
        
             }
        public void moveInput(double speed){
        outputMotor.set(-speed);
        inputMotor.set(-speed-.4);
                }
         public void stopFeed(){
                inputMotor.stopMotor();
        }
        public void stopOutput(){
                outputMotor.stopMotor();
                inputMotor.stopMotor();
        }
}
