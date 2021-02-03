package frc.robot.subsystems;

import frc.robot.shuffleboard.SmartDashboardWrapper;
import frc.robot.config.RobotMap;
import frc.robot.config.RobotMap.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotChassis implements RobotSubsystem {

    private Compressor        m_compressor;
    private Solenoidal        m_transmission;
    private CANEncoder        m_leftLeaderEnc, m_leftFollowerEnc, m_rightLeaderEnc, m_rightFollowerEnc;
    private CANSparkMax       m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower, m_rollers;
    private DoubleSolenoid    m_rollerSolenoids;
    private TeleopTransDrive  m_teleopTransDrive;
    //private LimelightDrive    m_limelightDrive;
    private IdleMode          m_lastIdleMode;
    //private Limelight         m_limelight;
    public RobotTurret        m_turret;
    public Invoked            m_invoked;
    public boolean            m_invoker;

    public RobotChassis(boolean invoker, XboxController controller) {
        //m_limelight = limelight;
        m_invoked = new Invoked();
        m_invoker = invoker;

        // Instantiate the compressor
        try {
            m_compressor = new Compressor(CompressorPort.MAIN_COMPRESSOR);
        } catch (Exception ex) {
            DriverStation.reportError("Could not start Compressor\n", false);
        }

        // Instantiate Drive Train Motors, Transmission, and also the Wrapper Drives
        try {
            m_transmission = new Solenoidal(SolenoidPort.DRIVE_TRANS_1, SolenoidPort.DRIVE_TRANS_2);
            
            double rampRate = SmartDashboard.getNumber("ramp rate", .25);

            
            m_leftLeader = new CANSparkMax(CANSparkID.LEFT_LEADER, MotorType.kBrushless);
            m_leftLeader.setInverted(true);
            m_leftLeader.setOpenLoopRampRate(rampRate);
            m_leftLeaderEnc = m_leftLeader.getEncoder();

            m_leftFollower = new CANSparkMax(CANSparkID.LEFT_FOLLOWER, MotorType.kBrushless);
            m_leftFollower.follow(m_leftLeader);
            m_leftFollowerEnc = m_leftFollower.getEncoder();

            m_rightLeader = new CANSparkMax(CANSparkID.RIGHT_LEADER, MotorType.kBrushless);
            m_rightLeader.setInverted(true);
            m_rightLeader.setOpenLoopRampRate(rampRate);
            m_rightLeaderEnc = m_rightLeader.getEncoder();

            m_rightFollower = new CANSparkMax(CANSparkID.RIGHT_FOLLOWER, MotorType.kBrushless);
            m_rightFollower.follow(m_rightLeader);
            m_rightFollowerEnc = m_rightFollower.getEncoder();
        

            DifferentialDrive rawDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
            m_teleopTransDrive = new TeleopTransDrive(rawDrive, m_transmission, PlayerButton.FORCE_LOW_TRANSMISSION, true);
            //m_limelightDrive = new LimelightDrive(rawDrive, m_transmission);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Drive Train Motors\n", false);
        }

        try {
            m_rollers = new CANSparkMax(CANSparkID.INTAKE_MOTOR, MotorType.kBrushed);
            m_rollerSolenoids = new DoubleSolenoid(RobotMap.SolenoidPort.ROLLER_IN, RobotMap.SolenoidPort.ROLLER_OUT);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate Intake Rollers\n", false);
        }
    }

    @Override
    public void displayOnShuffleboard() {
        SmartDashboardWrapper dashboardWrapper = new SmartDashboardWrapper(this);

        dashboardWrapper.putNumber("LeftLeader", m_leftLeaderEnc.getPosition());
        dashboardWrapper.putNumber("LeftFollower", m_leftFollowerEnc.getPosition());
        dashboardWrapper.putNumber("RightLeader", m_rightLeaderEnc.getPosition());
        dashboardWrapper.putNumber("RightFollower", m_rightFollowerEnc.getPosition());
        //dashboardWrapper.putNumber("PSI", m_compressor.getAirPressurePsi());
    }

    @Override
    public void updateFromShuffleboard() {
    }

    @Override
    public void teleopPeriodic(Joystick stick) {
        if (stick.getRawButton(PlayerButton.INTAKE_LIMELIGHT_DRIVE)) {
            //limelightPeriodic();
        } else {
            teleopPeriodic(stick, 1.0);
        }
    }

    public void teleopPeriodic(Joystick stick, Double abs_limit) {
        if (stick == null) {
            DriverStation.reportError("No Joystick, cannot run Chassis periodic\n", false);
            return;
        }

        //m_teleopTransDrive.curvatureDrive(stick, abs_limit);
        if(m_invoked.notInvoked(m_invoker, true)) {
            m_teleopTransDrive.arcadeDrive(stick, abs_limit);
        }

        if (m_compressor != null) {
            m_compressor.setClosedLoopControl(true);
        }
        
        if(m_rollers != null && stick.getRawButton(RobotMap.PlayerButton.INTAKE)){ 
            m_rollerSolenoids.set(Value.kForward);
        } else {
            m_rollerSolenoids.set(Value.kReverse);
        }
    }

    /*public void limelightPeriodic() {
        m_limelightDrive.autoDrive(m_limelight);
    }*/

    public boolean lowTransmission() {
        return m_teleopTransDrive.lowTransmission();
    }

    public boolean highTransmission() {
        return m_teleopTransDrive.highTransmission();
    }

    public CANSparkMax getLeftLeaderNeo() {
        return m_leftLeader;
    }

    public CANSparkMax getRightLeaderNeo() {
        return m_rightLeader; 
    }

    public void stop() {
        m_teleopTransDrive.stop();
    }

    public void setIdleMode(IdleMode mode) {
        if (m_lastIdleMode == null || m_lastIdleMode != mode) {
            System.out.println("Setting idle mode to "+mode);
            m_lastIdleMode = mode;
            m_leftLeader.setIdleMode(mode);
            m_leftFollower.setIdleMode(mode);
            m_rightLeader.setIdleMode(mode);
            m_rightFollower.setIdleMode(mode);
        }
    }

    public CANSparkMax rollers(){
        return m_rollers;
    }

    public double getLeft() {
        return m_leftLeader.get();
    }
    public double getRight() {
        return m_rightLeader.get();
    }
    public void setLeft(double speed) {
        m_leftLeader.set(speed);
    }
    public void setRight(double speed) {
        m_rightLeader.set(speed);
    }
    public boolean getSolenoids() {
        if(m_rollerSolenoids.get() == Value.kForward) {
            return true;
        } else {
            return false;
        }
    }
    public void setSolenoids(boolean extended) {
        Value value;
        if(extended == true) {
            value = Value.kForward;
        } else {
            value = Value.kReverse;
        }
        m_rollerSolenoids.set(value);
    }
    
}