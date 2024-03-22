// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    CommandSwerveDrivetrain m_drivetrain;
    Alliance alliance;
    private String ll = kCameraName;
    private Boolean enable = true;
    private Pose2d botpose;
    private static final RectanglePoseArea field =
            new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
    private Boolean hasTarget = false;

    /** Creates a new Limelight. */
    public Limelight(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        if (enable) {
            Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
            Double confidence = 1 - ((targetDistance - 1) / 6);
            LimelightHelpers.Results result =
            LimelightHelpers.getLatestResults(ll).targetingResults;
            if (result.valid) {
                botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);
                if (field.isPoseWithinArea(botpose)) {
                    if ((result.targets_Fiducials.length == 1 && targetDistance < 3.5)
                        || (result.targets_Fiducials.length == 2 && targetDistance < 6)
                        || (result.targets_Fiducials.length >= 3 && targetDistance < 8)) {
                        m_drivetrain.addVisionMeasurement(
                            botpose,
                            Timer.getFPGATimestamp()
                                - (result.latency_capture / 1000.0)
                                - (result.latency_pipeline / 1000.0),
                            VecBuilder.fill(confidence, confidence, .01));
                    }
                }
            }

            if (result.valid && LimelightHelpers.getTA(kCameraName) > .25) {
                hasTarget = true;
            } else {
                hasTarget = false;
            }
        }
    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public boolean hasTarget() {
        return hasTarget;
    }


}
