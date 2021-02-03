package frc.robot;

import frc.robot.subsystems.GunnerSubsystem;
import frc.robot.subsystems.RobotChassis;
import frc.robot.subsystems.RobotHood;
import frc.robot.config.RobotMap.*;
import frc.robot.subsystems.RobotShooter;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.RobotTurret;
import frc.robot.subsystems.RobotUtils.Limelight;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  Joystick     m_lightsaber;
  XboxController m_controller;
  RobotHood    m_hood;
  RobotShooter m_shooter;
  RobotTurret  m_turret;
  RobotChassis m_chassis;
  Limelight m_limelight;
  ArrayList<RobotSubsystem> m_robotSubsystems;
  ArrayList<GunnerSubsystem> m_gunnerSubsystems;

  @Override
  public void robotInit() {
    m_robotSubsystems = new ArrayList<RobotSubsystem>();
    m_gunnerSubsystems = new ArrayList<GunnerSubsystem>();
    
    try {
      m_lightsaber = new Joystick(ControllerPort.LIGHTSABER);
    } catch (Exception e) {
      DriverStation.reportError("Could not instantiate Joystick:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }

    try {
      m_controller = new XboxController(ControllerPort.GUNNER);
    } catch (Exception e) {
      DriverStation.reportError("Could not instantiate Controller:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }
    
    try {
      m_limelight = new Limelight();
    } catch (Exception e) {
      DriverStation.reportError("Could not instantiate Limelight:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }

    try {
      m_chassis = new RobotChassis(m_shooter.m_invokeDriver, m_controller);
      m_robotSubsystems.add(m_chassis);
    } catch (Exception e) {
      DriverStation.reportError("Could not instantiate Chassis:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }

    try {
      m_hood = new RobotHood();
      m_gunnerSubsystems.add(m_hood);
    } catch (Exception e) {
      DriverStation.reportError("Could not instantiate Hood:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }
    
    try {
      m_shooter = new RobotShooter(m_controller, CANSparkID.SHOOTER_LEFT, CANSparkID.SHOOTER_RIGHT, m_chassis);
      m_robotSubsystems.add(m_shooter);
    } catch (Exception e) {
      DriverStation.reportError("Could not instantiate Shooter:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }
    
    try {
      m_turret = new RobotTurret(m_limelight, false);
      m_gunnerSubsystems.add(m_turret);
    } catch (Exception e) {
      DriverStation.reportError("Could not in  istantiate Turret:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
    }
  }

  @Override
  public void robotPeriodic() {
    // Acquire limelight targets
    int pipeline = LimelightPipeline.POWER_PORT;
    if (m_lightsaber.getRawButton(PlayerButton.INTAKE_LIMELIGHT_DRIVE)) {
      pipeline = LimelightPipeline.PICKUP_POINT;
    }
    m_limelight.setPipelineMode(pipeline);
    m_limelight.updateTargets();

    // Update to/from the Shuffleboard
    for (RobotSubsystem subsystem : m_robotSubsystems) {
      subsystem.updateFromShuffleboard(); // Read first
      subsystem.displayOnShuffleboard(); // Display second
    }
    for (GunnerSubsystem gunnerSubsystem : m_gunnerSubsystems) {
      gunnerSubsystem.updateFromShuffleboard();
      gunnerSubsystem.displayOnShuffleboard();
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Main function that loops every 20ms for all of teleop. Wrap it in a try-catch and implement everything in teleopPeriodic_impl...
    for (RobotSubsystem subsystem : m_robotSubsystems) {
      try {
        subsystem.teleopPeriodic(m_lightsaber);
      } catch (Exception e) {
        DriverStation.reportError(e.toString(), false);
        e.printStackTrace();
      }
    }
    for (GunnerSubsystem gunnersubsystem : m_gunnerSubsystems) {
      try {
        gunnersubsystem.teleopPeriodic(m_controller);
      } catch (Exception e) {
        DriverStation.reportError(e.toString(), false);
        e.printStackTrace();
      }
    }

    //m_shooter.teleopPeriodic(m_lightsaber);
    //m_chassis.teleopPeriodic(m_lightsaber, 1.0);
    //m_hood.teleopPeriodic(m_lightsaber);
    //m_turret.teleopPeriodic(m_lightsaber);
  }


  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
  }
}
