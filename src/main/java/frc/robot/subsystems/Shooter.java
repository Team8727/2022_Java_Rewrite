package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shooterLead = new CANSparkMax(Constants.CanId.shooterLead, MotorType.kBrushless);
  private final CANSparkMax m_shooterFollow = new CANSparkMax(Constants.CanId.shooterFollow, MotorType.kBrushless);

  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(Constants.Shooter.Feedforward.kS, Constants.Shooter.Feedforward.kV, Constants.Shooter.Feedforward.kA);
  private final SparkMaxPIDController m_shooterPIDController;

  public Shooter() {
    m_shooterLead.restoreFactoryDefaults();
    m_shooterFollow.restoreFactoryDefaults();
    m_shooterLead.setInverted(Constants.Shooter.invert);
    m_shooterFollow.follow(m_shooterFollow, !Constants.Shooter.invert);
    m_shooterPIDController = m_shooterLead.getPIDController();

    m_shooterPIDController.setP(Constants.Shooter.PID.kP);
    m_shooterPIDController.setI(Constants.Shooter.PID.kI);
    m_shooterPIDController.setD(Constants.Shooter.PID.kD);
  }

  public void setVelocity(double velocity){
    m_shooterPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, m_shooterFeedforward.calculate(velocity));
  }

  public InstantCommand shooterIdleCommand(){
    return new InstantCommand(() -> this.setVelocity(Constants.Shooter.Velocities.idleVelocity), this);
  }

  public InstantCommand shooterSpinUpCommand(double velocity){
    return new InstantCommand(() -> this.setVelocity(velocity), this);
  }
}