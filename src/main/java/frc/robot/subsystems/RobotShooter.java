package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.config.*;
import frc.robot.shuffleboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class RobotShooter implements RobotSubsystem
{
    private CANSparkMax m_conveyer, m_cycler;
    private TalonSRX m_throat;
    private CANSparkMax m_leaderNeo, m_followerNeo;
    private CANEncoder m_neoEncoder;
    private CANPIDController m_neoPidController;

    private final double EPSILON = Constants.kShooterEpsilon;

    private boolean DEBUG = Constants.kShooterDebug;
    private boolean feed;
    public Invoked m_invoked;
    public boolean m_invokeDriver;

    public double kP = Constants.kShooterGains.kP;
    public double kI = Constants.kShooterGains.kI;
    public double kD = Constants.kShooterGains.kD;
    public double kIz = Constants.kShooterGains.kIz;
    public double kFF = Constants.kShooterGains.kFF;
    public double kMaxOutput = Constants.kShooterGains.kMaxOutput;
    public double kMinOutput = Constants.kShooterGains.kMinOutput;
    public double m_reference = Constants.kShooterRPM;
    public int throatPort = RobotMap.CANSparkID.THROAT_MOTOR;
    public int conveyerPort = RobotMap.CANSparkID.CONVEYER_MOTOR;
    public int cyclerPort = RobotMap.CANSparkID.CYCLER_MOTOR;
    public int rollerPort = RobotMap.CANSparkID.INTAKE_MOTOR;
    public RobotChassis m_chassis;
    public XboxController m_controller;

    public RobotShooter(XboxController controller, int leftNeoCanPort, int rightNeoCanPort, RobotChassis chassis) {
        this.m_chassis = chassis;
        m_leaderNeo = new CANSparkMax(rightNeoCanPort, MotorType.kBrushless);
        m_followerNeo = new CANSparkMax(leftNeoCanPort, MotorType.kBrushless);
        m_throat = new TalonSRX(throatPort);
        m_conveyer = new CANSparkMax(conveyerPort, MotorType.kBrushed);
        m_cycler = new CANSparkMax(cyclerPort, MotorType.kBrushed);
        m_followerNeo.follow(m_leaderNeo, true);
        m_invoked = new Invoked();
        m_controller = controller;

        m_neoEncoder = m_leaderNeo.getEncoder();

        m_neoPidController = m_leaderNeo.getPIDController();

        m_neoPidController.setP(kP);
        m_neoPidController.setI(kI);
        m_neoPidController.setD(kD);
        m_neoPidController.setIZone(kIz);
        m_neoPidController.setFF(kFF);
        m_neoPidController.setOutputRange(kMinOutput, kMaxOutput);

        m_neoPidController.setSmartMotionMaxVelocity(m_reference, 0);
        m_neoPidController.setSmartMotionMinOutputVelocity(0.0, 0);
        m_neoPidController.setSmartMotionMaxAccel(Constants.kShooterAcceleration, 0);
        m_neoPidController.setSmartMotionAllowedClosedLoopError(Constants.kShooterEpsilon, 0);

        m_leaderNeo.setIdleMode(IdleMode.kCoast);
        m_followerNeo.setIdleMode(IdleMode.kCoast);

        // m_outputEncoder = new DutyCycleEncoder(Sensor_Port.SHOOTERENCODER);

        // m_giver = new Talon(giverTalonPort);
    }

    @Override
    public void displayOnShuffleboard()
    {
        SmartDashboardWrapper dashboardWrap = new SmartDashboardWrapper(this);

        dashboardWrap.putNumber("RPM", getCurrentVelocity());
        dashboardWrap.putNumber("Target RPM", m_reference);
        dashboardWrap.putNumber("P Gain", kP);
        dashboardWrap.putNumber("I Gain", kI);
        dashboardWrap.putNumber("D Gain", kD);
        dashboardWrap.putNumber("I Zone", kIz);
        dashboardWrap.putNumber("Feed Forward", kFF);
        dashboardWrap.putNumber("Max Output", kMaxOutput);
        dashboardWrap.putNumber("Min Output", kMinOutput);
        dashboardWrap.putNumber("setReference", m_reference);
        dashboardWrap.putBoolean("feed", feed);
    }

    @Override
    public void updateFromShuffleboard() {
        SmartDashboardWrapper dashboardWrap = new SmartDashboardWrapper(this);
        // read PID coefficients from SmartDashboard
        double p = dashboardWrap.getNumber("P Gain", kP);
        double i = dashboardWrap.getNumber("I Gain", kI);
        double d = dashboardWrap.getNumber("D Gain", kD);
        double iz = dashboardWrap.getNumber("I Zone", kIz);
        double ff = dashboardWrap.getNumber("Feed Forward", kFF);
        double max = dashboardWrap.getNumber("Max Output", kMaxOutput);
        double min = dashboardWrap.getNumber("Min Output", kMinOutput);
        double minV = dashboardWrap.getNumber("Min Velocity", 0.0);
        double maxA = dashboardWrap.getNumber("Max Acceleration", Constants.kShooterAcceleration);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            m_neoPidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_neoPidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_neoPidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_neoPidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_neoPidController.setFF(ff);
            kFF = ff;
        }

        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_neoPidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        if ((minV != 0.0)) {
            m_neoPidController.setSmartMotionMinOutputVelocity(minV, 0);
            minV = 0.0;
        }
        if ((maxA != Constants.kShooterAcceleration)) {
            m_neoPidController.setSmartMotionMaxAccel(maxA, 0);
            maxA = (Constants.kShooterAcceleration);
        }

    }

    @Override
    public void teleopPeriodic(Joystick joystick) {
        /*if (DEBUG) {
            m_leaderNeo.set(joystick.getY());
            return;
        }*///

        if (m_controller.getTriggerAxis(Hand.kRight) != 0) {
            m_invokeDriver = true;
            fire();
        } else if (m_invoked.notInvoked(invokeDriver(), joystick.getRawButton(RobotMap.PlayerButton.INTAKE))) {
            m_chassis.rollers().set(Constants.kRollerSpeed);
            m_invokeDriver = false;
        } else {
            m_chassis.rollers().set(0.0);
            stop();
            m_invokeDriver = false;
        }
    }

    private void fire() {

        m_neoPidController.setReference(m_reference, ControlType.kSmartVelocity);
        feed();
    } // change

    private void stop() {
        stopShooter();
        stopFeeder();
    }

    public double getCurrentVelocity() {
        return m_neoEncoder.getVelocity();
    }

    // When the shooter is still spinning up (i.e. it's not at m_reference yet),
    // return false.
    // When it's ready to fire, return true.
    private void feed() {

        if (MathUtils.fuzzyEquals(m_neoEncoder.getVelocity(), m_reference, EPSILON)) {
            startFeeder();
            feed = true;
        } else {
            stopFeeder();
            feed = false;
        }
    }

    private void stopShooter() {
        
        m_leaderNeo.stopMotor();
    }

    private void startFeeder() {
        if (m_throat == null || m_conveyer == null || m_cycler == null || m_chassis == null) {
            return;
        }

            m_throat.set(ControlMode.PercentOutput, Constants.kthroatSpeed);
            m_conveyer.set(Constants.kConveyerSpeed);
            m_cycler.set(Constants.kCyclerSpeed);
            m_chassis.rollers().set(Constants.kRollerSpeed);
    }

    private void stopFeeder() {
        if (m_throat == null || m_conveyer == null || m_cycler == null || m_chassis == null) {
            return;
        }

        m_throat.set(ControlMode.PercentOutput, 0);
        m_conveyer.stopMotor();
        m_cycler.stopMotor();  
        m_chassis.rollers().set(0.0);
      
    }    

    public boolean invokeDriver() {
        return m_invokeDriver;
    }
    public double getShooter() {
        return getCurrentVelocity();
    }
    public void setShooter(double reference) {
        m_neoPidController.setReference(reference, ControlType.kSmartVelocity);
    }
    public double getThroat() {
        return m_throat.getMotorOutputPercent();
    }
    public void setThroat(double speed) {
        m_throat.set(ControlMode.PercentOutput, speed);
    }
    public double getConveyer() {
        return m_conveyer.get();
    }
    public void setConveyer(double speed) {
        m_conveyer.set(speed);
    }
    public double getCycler() {
        return m_cycler.get();
    }
    public void setCycler(double speed) {
        m_cycler.set(speed);
    }
    public double getRollers() {
        return m_chassis.rollers().get();
    }
}
