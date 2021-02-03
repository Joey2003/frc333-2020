package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import frc.robot.shuffleboard.*;
import frc.robot.config.Constants;
import frc.robot.config.RobotMap;
import edu.wpi.first.wpilibj.XboxController;

public class RobotHood implements GunnerSubsystem
{
    // motor and endoder attached to the shooter's hood
    private CANSparkMax m_hoodMotor;
    private CANEncoder m_hoodMotorEncoder;
    private CANPIDController m_pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, minVel, allowedErr, maxRPM, maxAcc, maxVel;
    public double kHoodmaxAcc,  kHoodmaxVel;
    private final double CUSP_POWER = Constants.kHoodCuspPower;
    double m_reference = 0.0;
    int absolutePos;

    public RobotHood() {

        // Instantiate Hood motor and encoder
        m_hoodMotor = new CANSparkMax(RobotMap.CANSparkID.HOOD_MOTOR, MotorType.kBrushed);
        m_hoodMotor.setInverted(true);
        m_hoodMotorEncoder = new CANEncoder(m_hoodMotor, EncoderType.kQuadrature, Constants.kHoodCPR);
        m_pidController = m_hoodMotor.getPIDController();

        kP = Constants.kHoodGains.kP;
        kI = Constants.kHoodGains.kI;
        kD = Constants.kHoodGains.kD;
        kIz = Constants.kHoodGains.kIz;
        kFF = Constants.kHoodGains.kFF;
        kMaxOutput = Constants.kHoodGains.kMaxOutput;
        kMinOutput = Constants.kHoodGains.kMinOutput;
        kHoodmaxAcc = Constants.kHoodmaxAcc;
        kHoodmaxVel = Constants.kHoodmaxVel;


        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        m_pidController.setSmartMotionMaxVelocity(kHoodmaxVel, Constants.kHoodSmartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(Constants.kHoodminVel, Constants.kHoodSmartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(kHoodmaxAcc, Constants.kHoodSmartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(Constants.kHoodAllowedErr, Constants.kHoodSmartMotionSlot);
    }

    @Override
    public void displayOnShuffleboard()
    {
        SmartDashboardWrapper dashboardWrapper = new SmartDashboardWrapper(this);
        dashboardWrapper.putNumber("Speed", getVelocity());
        dashboardWrapper.putNumber("Reading", getEncoderReading());
        dashboardWrapper.putNumber("setReference", m_reference);
    }

    @Override
    public void updateFromShuffleboard()
    {
        SmartDashboardWrapper dashboardWrapper = new SmartDashboardWrapper(this);
        // read PID coefficients from SmartDashboard
        double p    = dashboardWrapper.getNumber("P Gain", kP);
        double i    = dashboardWrapper.getNumber("I Gain", kI);
        double d    = dashboardWrapper.getNumber("D Gain", kD);
        double iz   = dashboardWrapper.getNumber("I Zone", kIz);
        double ff   = dashboardWrapper.getNumber("Feed Forward", kFF);
        double max  = dashboardWrapper.getNumber("Max Output", kMaxOutput);
        double min  = dashboardWrapper.getNumber("Min Output", kMinOutput);
        double maxV = dashboardWrapper.getNumber("Max Velocity", maxVel);
        double minV = dashboardWrapper.getNumber("Min Velocity", minVel);
        double maxA = dashboardWrapper.getNumber("Max Acceleration", maxAcc);
        double allE = dashboardWrapper.getNumber("Allowed Closed Loop Error", allowedErr);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            m_pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidController.setFF(ff);
            kFF = ff;
        }

        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        if ((maxV != maxVel)) {
            m_pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            m_pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            m_pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }

        if ((allE != allowedErr)) {
            m_pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }
    }

    @Override
    public void teleopPeriodic(XboxController lightsaber) {

        if (lightsaber.getRawButton(RobotMap.PlayerButton.INIT_LINE)) {
            m_reference = Constants.INIT_LINE_POSITION;
        } else if (lightsaber.getRawButton(RobotMap.PlayerButton.TARGET_ZONE)) {
            m_reference = Constants.TARGET_ZONE_POSTION;
        } else if (lightsaber.getRawButton(RobotMap.PlayerButton.FAR_TRENCH)) {
            m_reference = Constants.FAR_TRENCH_POSTITION;
        } else if (lightsaber.getRawButton(RobotMap.PlayerButton.NEAR_TRENCH)) {
            m_reference = Constants.NEAR_TRENCH_POSTION;
        }

        m_pidController.setReference(m_reference, ControlType.kSmartMotion);
    }

    public void resetEncoder() {
        m_hoodMotorEncoder.setPosition(0);
    }

    public double getEncoderReading() {
        return m_hoodMotorEncoder.getPosition();
    }

    public double getVelocity() {
        return m_hoodMotorEncoder.getVelocity();
    }

    public double get() {
        return m_hoodMotor.get();
    }

    public void set(double speed) {
        m_hoodMotor.set(speed);
    }

    public double getVoltage() {
        return m_hoodMotor.getBusVoltage();
    }

    public void cusp(double power) {
        m_hoodMotor.set(CUSP_POWER);
    }

    public void stop() {
        m_hoodMotor.stopMotor();
    }

    /*public boolean lowerThenPos(double pos, double lowerLimit, XboxController controller) {
        boolean dPad_up = controller.getPOV() < 90 && controller.getPOV() > 270;

        if (dPad_up && getEncoderReading() < pos && getEncoderReading() >= lowerLimit) {
            return true;
        } else {
            return false;
        }
    }

    public boolean greaterThenPos(double pos, double upperLimit, XboxController controller) {
        boolean dPad_down = controller.getPOV() > 90 && controller.getPOV() < 270;

        if (dPad_down && getEncoderReading() > pos && getEncoderReading() <= upperLimit) {
            return true;
        } else {
            return false;
        }
    }*/
}