package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.EdgeDetector;

import frc.robot.Constants;

/**
 * Intake subystem 
 * pneuamtics and conveyor stages
 */
public class Intake extends SubsystemBase{
	//Upper and lower stage positions
	public enum IndexPosition{
		UPPER,
		LOWER
	}

	private WPI_TalonSRX m_intakeLowerTalon = new WPI_TalonSRX(Constants.CanId.intakeLower);
	private WPI_TalonSRX m_upperConveyorTalon = new WPI_TalonSRX(Constants.CanId.upperConveyor);

	private DigitalInput m_lowerBreakBeam = new DigitalInput(Constants.Ports.lowerBreakBeamDIO);
	private DigitalInput m_upperBreakBeam = new DigitalInput(Constants.Ports.upperBreakBeamDIO);

	private EdgeDetector m_lowerBBDetector = new EdgeDetector(() -> !m_lowerBreakBeam.get(), Constants.breakBeamDebouce);
	private EdgeDetector m_upperBBDetector = new EdgeDetector(() -> !m_upperBreakBeam.get(), Constants.breakBeamDebouce);


	private Compressor m_compressor = new Compressor(Constants.CanId.compressor, PneumaticsModuleType.CTREPCM);
	private Solenoid m_intakeSolenoid = new Solenoid(Constants.CanId.compressor, PneumaticsModuleType.CTREPCM, Constants.Ports.solenoidPort);

	private boolean[] ballIndex = {false, false};

	//Constructor sets inverts
	public Intake(){
		m_intakeLowerTalon.setInverted(Constants.Intake.invertLower);
		m_upperConveyorTalon.setInverted(Constants.Intake.invertUpper);
	}

	@Override
	public void periodic(){
		SmartDashboard.putBooleanArray("What the fuck", ballIndex);
	}

	//Get the raw ball index
	public boolean[] getBallIndex(){
		return ballIndex;
	}

	//Get the ball index of a particular ball
	public boolean getBallIndex(IndexPosition position){
		if (position == IndexPosition.LOWER) return ballIndex[0];
		else return ballIndex[1];
	}

	//Set the ball index for a particular ball
	public void setBallIndex(IndexPosition position, boolean state){
		if (position == IndexPosition.LOWER) ballIndex[0] = state;
		else ballIndex[1] = state;
	}

	//Set the ball index for both balls
	public void setBallIndex(boolean lower, boolean upper){
		ballIndex[0] = lower;
		ballIndex[1] = upper;
	}

	//Check if breakbeam is broken
	public boolean getBreakBeam(IndexPosition position){
		if (position == IndexPosition.LOWER) return m_lowerBBDetector.get();
		else return m_upperBBDetector.get();
	}

	public boolean getBreakBeamRising(IndexPosition position){
		if (position == IndexPosition.LOWER) return m_lowerBBDetector.detectRising();
		else return m_upperBBDetector.detectRising();
	}

	//Set a conveyor motor's voltage
	public void setMotor(IndexPosition position, double voltage){
		if (position == IndexPosition.LOWER) m_intakeLowerTalon.setVoltage(voltage);
		else m_upperConveyorTalon.setVoltage(voltage);
	}

	//Set intake deployment
	public void setIntake(boolean position){
		m_intakeSolenoid.set(position);
	}

	//Set compressor enabled
	public void setCompressor(boolean enabled){
		if (enabled) m_compressor.enableDigital();
		else m_compressor.disable();
	}

	//Returns a basic command that doesn't use breakbeams to run the intake
	public Command sensorlessIntakeCommand(){
		return new StartEndCommand(
			() -> {
				setIntake(true);
				setMotor(IndexPosition.UPPER, Constants.Intake.OutputLevel.upperConveyorVoltage);
				setMotor(IndexPosition.LOWER, Constants.Intake.OutputLevel.intakeVoltage);
			},
			() -> {
				setIntake(false);
				setMotor(IndexPosition.UPPER, 0);
				setMotor(IndexPosition.LOWER, 0);
			},
			this);
	}

	public Command upperFeedCommand(){
		return new StartEndCommand(
			() -> {
				setMotor(IndexPosition.UPPER, Constants.Intake.OutputLevel.shooterFeedVoltage);
				if (getBallIndex(IndexPosition.LOWER)) setMotor(IndexPosition.LOWER, Constants.Intake.OutputLevel.shooterFeedVoltage);
			},
			() -> {
				setMotor(IndexPosition.UPPER, 0);
				setMotor(IndexPosition.LOWER, 0);
				setBallIndex(IndexPosition.UPPER, false);
			},
			this)
			.withTimeout(Constants.Intake.shootTime);
	}
	
	public Command indexBallsCommand(){
		return new StartEndCommand(
			() -> {
				setMotor(IndexPosition.UPPER, Constants.Intake.OutputLevel.upperConveyorVoltage);
				setMotor(IndexPosition.LOWER, Constants.Intake.OutputLevel.intakeVoltage);
			},
			() -> {
				setMotor(IndexPosition.UPPER, 0);
				setMotor(IndexPosition.LOWER, 0);
				setBallIndex(false, true);
			},
			this)
			.until(() -> getBreakBeamRising(IndexPosition.UPPER));//.withTimeout(Constants.Intake.indexingTimeout);
	}

	public Command ejectLowerCommand(){
		return new StartEndCommand(
			() -> setMotor(IndexPosition.LOWER, Constants.Intake.OutputLevel.ejectVoltage),
			() -> {
				setMotor(IndexPosition.LOWER, 0);
				setBallIndex(IndexPosition.LOWER, false);
			},
			this)
			.withTimeout(Constants.Intake.outtakeTimeout);
	}
}