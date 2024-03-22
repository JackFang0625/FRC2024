package frc.robot.Subsystems.Stage;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;

public class StageSubsystem extends SubsystemBase {

    // Initialize devices
    CANSparkMax m_stageMotor = new CANSparkMax(CanConstants.ID_StageMotor, MotorType.kBrushless);
    RelativeEncoder m_stageEncoder = m_stageMotor.getEncoder();
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
    boolean m_noteInStage = false;
    boolean m_stageRunning = false;

    /** Creates a new StageSubsystem. */
    public StageSubsystem() {

        // Set motor to factory defaults
        m_stageMotor.restoreFactoryDefaults();

        // Invert motor?
        m_stageMotor.setInverted(true);

        // Set motor to Brake
        m_stageMotor.setIdleMode(IdleMode.kCoast);

        // Config current limit
        // m_stageMotor.configSupplyCurrentLimit(new
        // SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));
    }

    @Override
    public void periodic() {

        // Check for change in beam break
        // Sensor returns true when beam NOT broken
        m_noteInStage = m_stageBeamBreak.get() ? false : true;

        SmartDashboard.putBoolean("Note In Stage?", m_noteInStage);

        if (RobotConstants.kIsStageTuningMode) {
            SmartDashboard.putNumber("Stage Current Draw", m_stageEncoder.getVelocity());
        }
    }

    /**
     * 
     * @param speed speed to set Stage motor at
     */
    public void runStage(double speed) {
        m_stageMotor.set(speed);
        m_stageRunning = true;
    }

    public void runStage() {
        m_stageMotor.set(StageConstants.kIntakeSpeed);
        m_stageRunning = true;
    }

    public void stopStage() {
        m_stageMotor.set(0);
        m_stageRunning = false;
    }

    // Do not use if the shooter's target velocity is zero.
    public void ejectFront(double speed) {
        this.runStage(speed);
    }

    public void ejectBack(double speed) {
        this.runStage((-1.0) * speed);
    }

    public boolean isNoteInStage() {
        return m_noteInStage;
    }

    public boolean isStageRunning() {
        return m_stageRunning;
    }

    /*
     * Command Factories
     */

    // Pass the Note to the Shooter
    public Command feedNote2ShooterCommand() {
        if (isNoteInStage()) {
            return new RunCommand(() -> this.ejectFront(StageConstants.kFeedToShooterSpeed), this)
                .until(() -> !isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());

        } else {
            // If there's no note to start, run with timeout
            return new RunCommand(() -> this.ejectFront(StageConstants.kFeedToShooterSpeed), this)
                .withTimeout(1.5) // run for 1.5 seconds
                .andThen(() -> this.stopStage());
        }
        
    }

    public Command feedStageCommand() {
        return new RunCommand(() -> this.ejectFront(.8), this)
                .until(() -> isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());
    }

}
