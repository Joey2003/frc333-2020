package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public interface RobotSubsystem {
    public void displayOnShuffleboard();
    public void updateFromShuffleboard();
    public void teleopPeriodic(Joystick joystick);
}

