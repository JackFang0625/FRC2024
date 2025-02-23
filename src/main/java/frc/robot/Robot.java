// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private Command m_lastAutonomousCommand;
    private RobotContainer m_robotContainer;
    private Field2d m_autoTraj = new Field2d();
    private List<Pose2d> m_pathsToShow = new ArrayList<Pose2d>();

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        m_robotContainer.m_drivetrain.getDaqThread().setThreadPriority(99);

        // This is the CTRE SignalLogger. It requires a USB drive in the RoboRIO
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/signal-logging.html
        // Don't use it for now
         SignalLogger.start();

         FollowPathCommand.warmupCommand().schedule(); //Added to warmup pathplanner class load
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.disablePIDSubsystems();
        SmartDashboard.putData("Auto Path Preview",m_autoTraj);
    }

    @Override
    public void disabledPeriodic() {
        // Get currently selected command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // Check if is the same as the last one
        if (m_autonomousCommand != m_lastAutonomousCommand) {
            // Check if its contained in the list of our autos
            if (AutoBuilder.getAllAutoNames().contains(m_autonomousCommand.getName())) {
                // Clear the current path
                m_pathsToShow.clear();
                // Grabs all paths from the auto
                for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(m_autonomousCommand.getName())) {
                    // Adds all poses to master list
                    m_pathsToShow.addAll(path.getPathPoses());
                }
                // Displays all poses on Field2d widget
                m_autoTraj.getObject("traj").setPoses(m_pathsToShow);
            }
        }
        m_lastAutonomousCommand = m_autonomousCommand;
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }   
        m_robotContainer.setTeleopHeadPID();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        //DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }


}
