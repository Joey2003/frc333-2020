package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.config.*;

import frc.robot.subsystems.RobotUtils.Limelight;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * 
 * Add your docs here.
 * 
 */

public class RobotTurret implements GunnerSubsystem

{

    private TalonSRX m_turretMotor;

    public boolean m_reverse;
    public double tx;
    public double tv;
    public boolean m_invokeDriver;

    public RobotTurret(Limelight lime, boolean reverse) {

        m_turretMotor = new TalonSRX(RobotMap.TalonPort.TURRET_MOTOR);

        m_turretMotor.setNeutralMode(NeutralMode.Brake);

        m_reverse = reverse;
        tv = lime.get_tv();
        tx = lime.get_tx();

        // Smart Motion Coefficients

    }

    @Override
    public void displayOnShuffleboard() {

    }

    @Override
    public void updateFromShuffleboard() {

    }

    @Override
    public void teleopPeriodic(XboxController lightSaber) {

        if (lightSaber.getTriggerAxis(Hand.kLeft) != 0) {

            track(tx, tv, .3);
        } else if (lightSaber.getRawAxis(0) != 0) {

            m_invokeDriver = false;
            pan((lightSaber.getTriggerAxis(Hand.kLeft) != 0 && tv == 1), lightSaber.getRawAxis(0) * .2);
            lightSaber.setRumble(RumbleType.kRightRumble, Math.abs(lightSaber.getRawAxis(0)));
        } else {

            m_invokeDriver = false;
            stop();
            lightSaber.setRumble(RumbleType.kRightRumble, 0.0);
            lightSaber.setRumble(RumbleType.kLeftRumble, 0.0);

        }

    }

    public double getOutput() {

        return m_turretMotor.getMotorOutputPercent();

    }

    public double getVoltage() {

        return m_turretMotor.getBusVoltage();

    }

    public void stop() {

        m_turretMotor.set(ControlMode.PercentOutput, 0.0);

    }

    // COMMENT ON CHIEF DELFI BY TheMrGerb:

    // Personally, what I have is a turret that runs on Motion Magic, and it takes

    // the tx, checks whether its a positive or negative value, and multiplies it by

    // 4096/360(the amount of encoder counts per degree). If the tx is negative, it

    // subtracts the offset from the current position and if the tx is positive, it

    // adds it. Hope this helps!

    public void track(double tx, double tv, double speed) {

        double degreeCPR = Constants.kTurretCPR / 360;

        double offset = degreeCPR * tx;

        double multiplier = (offset < 0 ? m_reverse ? 1 : -1 : m_reverse ? -1 : 1);

        if (tv == 1) {

            m_turretMotor.set(ControlMode.PercentOutput, speed * multiplier);
            m_invokeDriver = true;

        }

    }

    public void pan(boolean invoker, double speed) {

        if (invoker == false) {
            m_turretMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public boolean invokeDriver() {
        return m_invokeDriver;
    }

    public double get() {
        return m_turretMotor.getMotorOutputPercent();
    }
    public void set(double demand) {
        m_turretMotor.set(ControlMode.PercentOutput, demand);
    }

}