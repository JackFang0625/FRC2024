
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {

    /* Initialize Talons */
    CANSparkMax m_intakeMotor = new CANSparkMax(CanConstants.ID_IntakeMotor, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();

    /* Flag to tell if intake is running */
    private boolean m_intakeRunning = false;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        // Set motor to factory defaults
        m_intakeMotor.restoreFactoryDefaults();

        // Invert motor?
        m_intakeMotor.setInverted(true);

        // Set motor to Brake
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if (RobotConstants.kIsIntakeTuningMode) {
            SmartDashboard.putNumber("Intake Current Draw", m_intakeEncoder.getVelocity());
            // SmartDashboard.putNumber("Intake Center Current Draw",
            // m_centeringMotor.getSupplyCurrent());
        }
    }

    /**
     * 
     * @param speed speed to set intake motor at (-1,1)
     */
    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
        m_intakeRunning = true;
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
        m_intakeRunning = false;
    }

    public boolean isIntakeRunning() {
        return m_intakeRunning;
    }
    
    /*
     * Command Factories
     */
    public Command runIntakeCommand() {
        return new RunCommand(() -> this.runIntake(IntakeConstants.kIntakeSpeed), this);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> this.stopIntake(), this);
    }

    public Command ejectIntakeCommand() {
        return new RunCommand(() -> this.runIntake(IntakeConstants.kEjectSpeed), this);
    }

}
