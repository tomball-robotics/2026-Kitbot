package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDrive extends SubsystemBase{
    private SparkMax leftLeader;
  private SparkMax leftFollower;
  private SparkMax rightLeader;
  private SparkMax rightFollower;
private DifferentialDrive m_robotDrive;

  private SparkMaxConfig globalConfig = new SparkMaxConfig();
  private SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
  private SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  private SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
  private SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
  
  private Supplier<Double> leftAxis;
  private Supplier<Double> rotationAxis;
    public TankDrive(Supplier<Double> forward, Supplier<Double> rotate){
      leftAxis = forward;
      rotationAxis = rotate;

//public TankDrive(){


      leftLeader = new SparkMax(Constants.DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
        leftFollower = new SparkMax(Constants.DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
        rightLeader = new SparkMax(Constants.DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
        rightFollower = new SparkMax(Constants.DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);
m_robotDrive = new DifferentialDrive(leftLeader::set, rightLeader::set);
      globalConfig.smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
      leftLeaderConfig.apply(globalConfig)
      .inverted(false);
      leftFollowerConfig.apply(globalConfig).follow(leftLeader);
      
      rightLeaderConfig.apply(globalConfig)
      .inverted(true);
      rightFollowerConfig.apply(globalConfig)
      .follow(rightLeader);
        
        leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 
    @Override
    public void periodic(){
      /* 
      double forwardVal = leftAxis.get();
      double rotateVal = -rotationAxis.get();
      leftLeader.set(forwardVal+ rotateVal);
      rightLeader.set(forwardVal- rotateVal);*/
        m_robotDrive.arcadeDrive(leftAxis.get(), rotationAxis.get());
        
        
    } 
}
    /* 
    public void move(double speed, double turn){
      leftLeader.set(speed+ turn);
      leftFollower.set(speed + turn);
      rightLeader.set(speed-turn);
      rightFollower.set(speed-turn);
    }
    
    public void stop(){
      leftLeader.set(0);
      leftFollower.set(0);
      rightLeader.set(0);
      rightFollower.set(0);
    }
      
}
*/