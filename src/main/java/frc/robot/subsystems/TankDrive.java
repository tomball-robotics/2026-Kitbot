package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;


public class TankDrive extends SubsystemBase{
  private PIDController turnPID;
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
      turnPID = new PIDController(1, 0, 0);
      turnPID.setTolerance(.05);
      
      leftLeader = new SparkMax(Constants.DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
        leftFollower = new SparkMax(Constants.DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
        rightLeader = new SparkMax(Constants.DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
        rightFollower = new SparkMax(Constants.DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);
m_robotDrive = new DifferentialDrive(leftLeader::set, rightLeader::set);
      globalConfig.smartCurrentLimit(40)
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
    public Command pointToHub(){
      return this.run( () -> {
        LimelightHelpers.PoseEstimate position = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
        if(position.tagCount >= 1){
                  

        double robotX = position.pose.getX();
        double robotY = position.pose.getY();
        double dx = 11.75 - robotX;
        double dy = 4.5 - robotY;
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy,dx));
        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);
        SmartDashboard.putNumber("Target angle (deg)", targetAngle.plus(Rotation2d.k180deg).getDegrees());
        SmartDashboard.putNumber("Robot angle (deg)", position.pose.getRotation().getDegrees());
        }
        
        });
      }
      
      public Command turnToHub(){
        return this.run( () -> {
        LimelightHelpers.PoseEstimate position = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
        if(position.tagCount >= 1){
                  

        double robotX = position.pose.getX();
        double robotY = position.pose.getY();
        double dx = 11.75 - robotX;
        double dy = 4.5 - robotY;
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy,dx));
        Rotation2d change = targetAngle.plus(Rotation2d.k180deg).minus(position.pose.getRotation());
        SmartDashboard.putNumber("Change", change.getDegrees());
        turnPID.setSetpoint(targetAngle.plus(Rotation2d.k180deg).getRadians());
        double turnValue = turnPID.calculate(position.pose.getRotation().getRadians());
        turnValue = Math.min(.85,Math.abs(turnValue))*Math.signum(turnValue);
        if(turnValue>0.3){
        turnValue = Math.max(.5,Math.abs(turnValue))*Math.signum(turnValue);
        }
        SmartDashboard.putNumber("Turn Value", turnValue);
        m_robotDrive.arcadeDrive(leftAxis.get(), -turnValue);
        /* 
        if(Math.abs(change.getDegrees()) > 40){
          m_robotDrive.arcadeDrive(leftAxis.get(), -change.getDegrees()/45);
        }
        else if(Math.abs(change.getDegrees()) > 5){
          m_robotDrive.arcadeDrive(leftAxis.get(), -change.getDegrees()/50);
        }
        else{
          m_robotDrive.arcadeDrive(leftAxis.get(), 0);
        }
        */
        }
        
        });
      
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