package frc.robot.Subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.Setpoints;

public class ShooterSubsystem extends SubsystemBase {

    /* Hardware */
    CANSparkMax m_motorLeft = new CANSparkMax(CanConstants.ID_ShooterLeft, MotorType.kBrushless);
    CANSparkMax m_motorRight = new CANSparkMax(CanConstants.ID_ShooterRight, MotorType.kBrushless);

    RelativeEncoder m_encoderLeft = m_motorLeft.getEncoder();
    RelativeEncoder m_encoderRight = m_motorRight.getEncoder();

    SparkPIDController m_pidLeft = m_motorLeft.getPIDController();
    SparkPIDController m_pidRight = m_motorRight.getPIDController();

    /*
     * Setup the Velocity PID control object Start at velocity 0, disable FOC, no feed forward, use slot 0
     */
    private final double kP = 0, kI = 0, kD = 0, kIz = 0, kFF = 0.000179,  kMinOutput = -0.5, kMaxOutput = 1;

    // Shooter Velocity setpoints (in RPS)
    private double m_ShooterSetpointL = 0, m_ShooterSetpointR = 0;

    // Which side of the shooter?
    public enum kShooterSide {
        kLEFT,
        kRIGHT
    };

    // Shooter state
    public enum kShooterState {
        kOFF,
        kIDLE,
        kSPOOLING,
        kREADY
    }
    kShooterState m_state = kShooterState.kOFF;

    double m_idleSpeed = ShooterConstants.kShooterIdleSpeed;

    public ShooterSubsystem() {
        m_motorLeft.restoreFactoryDefaults();
        m_motorRight.restoreFactoryDefaults();

        m_motorLeft.setInverted(false);
        m_motorRight.setInverted(true);

        /* set motors to Coast */
        m_motorLeft.setIdleMode(IdleMode.kCoast);
        m_motorRight.setIdleMode(IdleMode.kCoast);

        /* Config the peak outputs */
        m_pidLeft.setOutputRange(kMinOutput, kMaxOutput);
        m_pidRight.setOutputRange(kMinOutput, kMaxOutput);

        /* Update Shooter Gains from TunableNumbers */
        m_pidLeft.setP(kP);
        m_pidLeft.setI(kI);
        m_pidLeft.setD(kD);
        m_pidLeft.setIZone(kIz);
        m_pidLeft.setFF(kFF);

        m_pidRight.setP(kP);
        m_pidRight.setI(kI);
        m_pidRight.setD(kD);
        m_pidRight.setIZone(kIz);
        m_pidRight.setFF(kFF);
    }

    @Override
    public void periodic() {
 
        boolean wheelsAtSpeed = areWheelsAtSpeed();
        SmartDashboard.putBoolean("Shooter at Speed?", wheelsAtSpeed);

        if (RobotConstants.kIsShooterTuningMode) {
            // Put actual velocities to smart dashboard
            SmartDashboard.putNumber("Shooter Velocity L", getShooterVelocity(kShooterSide.kLEFT));
            SmartDashboard.putNumber("Shooter Velocity R", getShooterVelocity(kShooterSide.kRIGHT));
        }

        if (m_state == kShooterState.kSPOOLING && wheelsAtSpeed) {
            // If Shooter is spooling up, signal when it's at speed 
            m_state = kShooterState.kREADY;

        } else if (m_state == kShooterState.kREADY && !wheelsAtSpeed) {
            // If Shooter is ready, signal if it falls off the target speed 
            m_state = kShooterState.kSPOOLING;
        }
    }

    /**
     * @param targetVelocity the velocity in RPS of the shooter Right
     * @param targetVelocityL the velocity in RPS of the shooter Left
     */
    public void runShooter(double targetVelocityL, double targetVelocityR) {
        // Save Velocity setpoints
        m_ShooterSetpointL = targetVelocityL;
        m_ShooterSetpointR = targetVelocityR;
        m_pidLeft.setReference(targetVelocityL, CANSparkMax.ControlType.kVelocity);
        m_pidRight.setReference(targetVelocityR, CANSparkMax.ControlType.kVelocity);
        if (m_state != kShooterState.kREADY) {
            m_state = kShooterState.kSPOOLING;
        }
    }

    public void runShooter() {
        // Get Velocity setpoint from TunableNumber
        m_pidLeft.setReference(m_ShooterSetpointL, CANSparkMax.ControlType.kVelocity);
        m_pidRight.setReference(m_ShooterSetpointR, CANSparkMax.ControlType.kVelocity);
        if (m_state != kShooterState.kREADY) {
            m_state = kShooterState.kSPOOLING;
        }
    }

    public void stopShooter() {
        m_motorLeft.set(0);
        m_motorRight.set(0);
        m_state = kShooterState.kOFF;
    }

    public void runIdle() {
        m_ShooterSetpointL = m_idleSpeed;
        m_ShooterSetpointR = m_idleSpeed;
        m_pidLeft.setReference(m_ShooterSetpointL, CANSparkMax.ControlType.kVelocity);
        m_pidRight.setReference(m_ShooterSetpointR, CANSparkMax.ControlType.kVelocity);
        m_state = kShooterState.kIDLE;
    }

    public void setIdleShooterSpeed(double speed) {
        m_idleSpeed = speed;
    }


    /**
     * @param setpoints - Reference to a Setpoints class instance
     */
    public void setShooterSetpoints(Setpoints setpoints) {
        m_ShooterSetpointL = setpoints.shooterLeft;
        m_ShooterSetpointR = setpoints.shooterRight;
    }

    /**
     * @param int side - the side of the shooter to query (0 = left, 1 = right)
     * @return the velocity of the specified shooter side in RPS
     */
    public double getShooterVelocity(kShooterSide side) {
        switch(side) {
        case kLEFT:
            return m_encoderLeft.getVelocity();
        case kRIGHT:
            return m_encoderRight.getVelocity();
        default:
            return 0.0;
        }
    }

    /**
     * Private method for subsystem use only. Outside callers should use isShooterAtSpeed().
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean areWheelsAtSpeed() {
        double leftErr = Math.abs(m_ShooterSetpointL - getShooterVelocity(kShooterSide.kLEFT));
        double rightErr = Math.abs(m_ShooterSetpointR - getShooterVelocity(kShooterSide.kRIGHT));
        return (leftErr + rightErr / 2.0) < ShooterConstants.kShooterTolerance;

    }

    /**
     * Quicker method for outside callers (i.e. LEDSubsystem)
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean isShooterAtSpeed () {
        return (m_state == kShooterState.kREADY);
    }

    /*
     * Command Factories
     */
    public Command runShooterCommand(double velocityL, double velocityR) {
        return new RunCommand(()->this.runShooter(velocityL, velocityR), this);
    }

    public Command runShooterCommand() {
        return new RunCommand(()->this.runShooter(), this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(()->this.stopShooter(), this);
    }
}