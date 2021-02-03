package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

public interface GunnerSubsystem {
    public void displayOnShuffleboard();
    public void updateFromShuffleboard();
    public void teleopPeriodic(XboxController joystick);
}

