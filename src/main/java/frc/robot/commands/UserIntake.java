package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IndexPosition;
import edu.wpi.first.util.WPIUtilJNI;

import frc.robot.Constants;

/**
 * User Intake command
 * Stateful command that takes in an input supplier used to turn off the intake. 
 * (This should be done with the end() method, but due to the possibility of balls in the system that haven't been indexed that isn't possible)
 */
public class UserIntake extends CommandBase{
  private final Intake m_intake;
  private final BooleanSupplier m_intakeInput;
  private IntakeState m_state = IntakeState.BOTH;
  private boolean m_userEngaged = false;
  private double m_endTime;

  //Possible states for the intake command
  enum IntakeState{
    BOTH,
    LOWER,
    INDEXING,
    FINISHED
  }

  //Constructs the object
  public UserIntake(BooleanSupplier intakeInputSupplier, Intake intake){
    m_intakeInput = intakeInputSupplier;
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize(){
    //Set to known state
    m_state = IntakeState.BOTH;
    m_userEngaged = false;

    //Exit command if both balls are indexed, or start in the lower intake state if one ball is indexed
    if (m_intake.getBallIndex(IndexPosition.LOWER)){
      m_state = IntakeState.FINISHED;
      return;
    }
    else if (m_intake.getBallIndex(IndexPosition.UPPER)) m_state = IntakeState.LOWER;

    //Deploy intake
    m_intake.setIntake(true);

    //Select starting configuration for motors
    if (m_state == IntakeState.BOTH) m_intake.setMotor(IndexPosition.UPPER, Constants.Intake.OutputLevel.upperConveyorVoltage);
    m_intake.setMotor(IndexPosition.LOWER, Constants.Intake.OutputLevel.intakeVoltage);
  }

  @Override
  public void execute(){
    //Check for button press to toggle intake engagement (begins as false as initial button press to begin the command will cause this to enable)
    if (m_intakeInput.getAsBoolean()) m_userEngaged = !m_userEngaged;

    //Move from intaking both stages to the lower stage if a ball is detected in the upper stage
    if (m_state == IntakeState.BOTH && m_intake.getBreakBeam(IndexPosition.UPPER)){
      m_state = IntakeState.LOWER;
      m_intake.setBallIndex(IndexPosition.UPPER, true);
      m_intake.setMotor(IndexPosition.UPPER, 0);
    }
    //Move from intaking lower to finished state if a ball is detected in the lower stage
    else if (m_state == IntakeState.LOWER && m_intake.getBreakBeam(IndexPosition.LOWER)){
      m_state = IntakeState.FINISHED;
      m_intake.setBallIndex(IndexPosition.LOWER, true);
      m_intake.setMotor(IndexPosition.LOWER, 0);
      m_intake.setIntake(false);
    }

    //If intake is unengaged and not already in the indexing state move to the indexing state and begin the timeout
    if (!m_userEngaged && m_state != IntakeState.INDEXING){
      m_state = IntakeState.INDEXING;
      m_endTime = WPIUtilJNI.now() * 1e-6 + Constants.Intake.intakeTimeout;
      m_intake.setIntake(false);
    }
    //If in the indexing state and the intake is rengaged move back to the appropriate intaking state
    else if (m_userEngaged && m_state == IntakeState.INDEXING){
      m_intake.setIntake(true);
      if (m_intake.getBallIndex(IndexPosition.UPPER)) m_state = IntakeState.LOWER;
      else m_state = IntakeState.BOTH;
    }

    //Move to finished state if indexing timeout is reached
    if (m_state == IntakeState.INDEXING && WPIUtilJNI.now() * 1e-6 >= m_endTime){
      m_state = IntakeState.FINISHED;
      m_intake.setMotor(IndexPosition.UPPER, 0);
      m_intake.setMotor(IndexPosition.LOWER, 0);
    }
  }

  @Override
  public void end(boolean interrupted){
    //Redundant set statements to ensure ending state
    m_intake.setMotor(IndexPosition.UPPER, 0);
    m_intake.setMotor(IndexPosition.LOWER, 0);
    m_intake.setIntake(false);
  }

  @Override
  public boolean isFinished(){
    //Exit command when finished state is reached
    if (m_state == IntakeState.FINISHED) return true;
    return false;
  }
}