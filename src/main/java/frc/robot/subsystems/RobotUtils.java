package frc.robot.subsystems;

import frc.robot.config.RobotMap;
import frc.robot.config.RobotMap.*;
import frc.robot.subsystems.RobotUtils.Limelight;

import java.util.ArrayDeque;
import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotUtils {

    public static boolean dbl_equals_power(double a, double b, int precision) {
        return Math.abs(a - b) <= Math.pow(10, -precision);
    }

    public static boolean dbl_equals(double a, double b, double precision) {
        return Math.abs(a - b) <= precision;
    }

    public static double abs_min(double a, double abs_b) {
        double sign = (a < 0 ? -1 : 1);
        return Math.min(Math.abs(a), abs_b) * sign;
    }

    public static class Limelight
    {
        private NetworkTable m_limelight;
        private NetworkTableEntry m_limelightPipeline;
        private MjpegServer m_server;

        private double m_tx, // x
                       m_ty, // y
                       m_ta, // area
                       m_ts, // skew
                       m_tv; // valid

        public Limelight()
        {
            m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
            m_limelightPipeline = m_limelight.getEntry("pipeline");

            try {
                m_server = new MjpegServer("limelight", "limelight.local", 5800);
              } catch (Exception e) {
                DriverStation.reportError("Could not instantiate limelight Stream:"+e.getMessage()+" ("+e.getStackTrace().toString()+")", false);
              }
        }

        public void updateTargets()
        {
            m_tx = m_limelight.getEntry("tx").getDouble(0.0);
            m_ty = m_limelight.getEntry("ty").getDouble(0.0);
            m_ta = m_limelight.getEntry("ta").getDouble(0.0);
            m_ts = m_limelight.getEntry("ts").getDouble(0.0);
            m_tv = m_limelight.getEntry("tv").getDouble(0.0);
        }

        public double get_tx() { return m_tx; }
        public double get_ty() { return m_ty; }
        public double get_ta() { return m_ta; }
        public double get_ts() { return m_ts; }
        public double get_tv() { return m_tv; }


        public void setPipelineMode(int mode)
        {
            int current_pipeline_index = m_limelightPipeline.getNumber(-1).intValue();
            int new_pipeline_index = mode;
            if (new_pipeline_index != current_pipeline_index) {
                System.out.println("Setting Limelight Pipeline from "+current_pipeline_index+" to "+new_pipeline_index);
                m_limelightPipeline.setNumber(new_pipeline_index);
            }
        }

    }

    public static class LimelightLED {
        /* Configurables */
        public static final int END_BUFFER_TIME   = 2000;
        public static final int DRIVE_BUFFER_TIME = 1250;

        /* No touchy below here. */
        private int m_lastLimelightLedMode = -1;
        private NetworkTableEntry m_limelightLedMode = null;
        private Integer m_turnOffBuffer;
        private long m_lastSetTime = 0;
        private Long m_lastTurnedOnTime = null;

        public LimelightLED(NetworkTableEntry limelightLedMode) {
            m_limelightLedMode = limelightLedMode;
            if (RobotMap.LimelightConservativeLED.isDoomsday) {
                m_turnOffBuffer = END_BUFFER_TIME;
            }
        }

        public void set(int mode) { set(mode, false); }
        
        public void set(int mode, boolean ignoreTurnOffBuffer) {
            if (m_lastLimelightLedMode != mode) {
                long now = System.currentTimeMillis();
                if (mode == RobotMap.LimelightLEDMode.OFF // Are we trying to turn off the light?
                    && m_turnOffBuffer != null            // ...and do we have a "Turn Off" buffer because we're in Doomsday Mode?
                    && !ignoreTurnOffBuffer               // ...and has the caller not explicitly told us to ignore the "Turn Off" buffer?
                    && (now - m_lastSetTime) < m_turnOffBuffer.intValue()) // ...and has the "Turn Off" buffer not yet been passed?
                { // If all of that was true, don't actually do anything!
                    return;
                }
                
                // We only reach this code if we're specifically *changing* the mode!
                m_limelightLedMode.setNumber(mode);
                m_lastLimelightLedMode = mode;
                if (mode == RobotMap.LimelightLEDMode.ON) {
                    m_lastTurnedOnTime = System.currentTimeMillis();
                } else {
                    m_lastTurnedOnTime = null;
                }
            } 
            m_lastSetTime = System.currentTimeMillis();
        }

        public boolean hasLightBeenOnLongEnoughForAutoDrive() {
            long now = System.currentTimeMillis();
            if (m_lastTurnedOnTime != null
                && (now - m_lastTurnedOnTime > DRIVE_BUFFER_TIME)) {
                return true;
            }
            return false;
        }
    }
}



class SolenoidT {
    private Solenoid m_solenoid;
    private Long m_lastFalse, m_lastTrue;

    public SolenoidT(int id) {
        m_solenoid = new Solenoid(id);
    }

    public void set(boolean value) {
        boolean last = m_solenoid.get();
        m_solenoid.set(value);
        if (last != value) {
            long now = System.currentTimeMillis();
            if (value) {
                m_lastTrue = now;
            } else {
                m_lastFalse = now;
            }
        }
    }

    public Long getLastBecameTrue() {
        return m_lastTrue;
    }

    public Long getLastBecameFalse() {
        return m_lastFalse;
    }
}

class Solenoidal {
    // Solenoids always come in pairs that are inverses of each other.
    // A "Solenoidal" is a wrapper around the solenoids to represent the two as
    // a single entity.
    private Solenoid m_solenoid1, m_solenoid2;
    private boolean m_state;

    public Solenoidal(int id1, int id2) {
        m_solenoid1 = new Solenoid(id1);
        m_solenoid2 = new Solenoid(id2);
    }

    public void set(boolean value) {
        m_state = value;
        m_solenoid1.set(m_state);
        m_solenoid2.set(!m_state);
    }

    public boolean get() {
        return m_state;
    }
}

class MultiSolenoidal {
    private ArrayList<Solenoidal> m_solenoidals;

    public MultiSolenoidal(Solenoidal... solenoidals) {
        m_solenoidals = new ArrayList<Solenoidal>();
        for (Solenoidal solenoidal : solenoidals) {
            m_solenoidals.add(solenoidal);
        }
    }

    public void set(boolean value) {
        for (Solenoidal solenoidal : m_solenoidals) {
            solenoidal.set(value);
        }
    }
}

class MultiCANEncoder {
    private ArrayList<CANEncoder> m_encoders;

    public MultiCANEncoder(CANSparkMax... neos) {
        m_encoders = new ArrayList<CANEncoder>();
        for (CANSparkMax neo : neos) {
            m_encoders.add(neo.getEncoder());
        }
    }

    public double getAverageVelocity() {
        double sum = 0.0;
        for (CANEncoder enc : m_encoders) {
            sum += enc.getVelocity();
        }

        return sum / m_encoders.size();
    }
}

class MultiCANPIDController {
    private ArrayList<CANPIDController> m_pids;

    public MultiCANPIDController(CANSparkMax... neos) {
        m_pids = new ArrayList<CANPIDController>();
        for (CANSparkMax neo : neos) {
            m_pids.add(neo.getPIDController());
        }

        // PID coefficients. Stolen from someone else's configuration on the internet.
        // We'll probably need to tweak these a bit :)
        final double kP = 0.1;
        final double kI = 1e-4;
        final double kD = 1;
        final double kIz = 0;
        final double kFF = 0;
        final double kMaxOutput = 1;
        final double kMinOutput = 0;

        for (CANPIDController pid : m_pids) {
            pid.setP(kP);
            pid.setI(kI);
            pid.setD(kD);
            pid.setIZone(kIz);
            pid.setFF(kFF);
            pid.setOutputRange(kMinOutput, kMaxOutput);
        }
    }

    public void setReference(double value) {
        for (CANPIDController pid : m_pids) {
            pid.setReference(value, ControlType.kSmartVelocity);
        }
    }
}

class UltrasonicMonitor implements Runnable {
    Ultrasonic m_ultrasonic;

    public UltrasonicMonitor(Ultrasonic ultrasonic) {
        m_ultrasonic = ultrasonic;
    }

    @Override
    public void run() {
        while (m_ultrasonic != null) {
            SmartDashboard.putNumber("Ultrasonic", m_ultrasonic.getRangeInches());
            Timer.delay(0.05);
        }
    }
}

class ArmPotentiometerMonitor implements Runnable {
    AnalogPotentiometer m_armPotentiometer;

    public ArmPotentiometerMonitor(AnalogPotentiometer potentiometer) {
        m_armPotentiometer = potentiometer;
    }

    @Override
    public void run() {
        while (m_armPotentiometer != null) {
            SmartDashboard.putNumber("Arm Potentiometer", m_armPotentiometer.get());
            Timer.delay(0.05);
        }
    }
}

class LimitSwitchMonitor implements Runnable {
    DigitalInput m_limitSwitch;

    public LimitSwitchMonitor(DigitalInput limitSwitch) {
        m_limitSwitch = limitSwitch;
    }

    @Override
    public void run() {
        while (m_limitSwitch != null) {
            SmartDashboard.putBoolean("Limit Switch Value", m_limitSwitch.get());
            Timer.delay(0.05);
        }
    }
}

class DummyAHRS {
    public DummyAHRS(Port port) {
    }

    public void reset() {
        appendToHistory(0.0);
        averageHistory();
    }

    public void resetDisplacement() {
    }

    public double getAngle() {
        return 123.45;
    }

    private double averageHistory() {
        double sum = 0.0;
        for (int i = 0; i < m_transHistory.size(); i++) {
            sum += m_transHistory.get(i);
        }
        return sum / m_transHistory.size();
    }

    private void appendToHistory(double data) {
        m_transHistory.add(data);
        if (m_transHistory.size() > HISTORY_LIMIT) {
            m_transHistory.remove(0);
        }
    }

    private ArrayList<Double> m_transHistory;
    private static double HISTORY_LIMIT = 20;
    public static double MAX_RANGE = 4800.0;
}


class MultiLimitSwitch {
    private DigitalInput[] m_limitSwitches;

    public MultiLimitSwitch(DigitalInput... limitSwitches) {
        m_limitSwitches = limitSwitches;
    }

    public boolean get()
    {
        for (DigitalInput limitSwitch : m_limitSwitches) {
            if (limitSwitch != null && !limitSwitch.get()) {
                return false;
            }
        }
        return true;
    }
}

class MultiSpeedController implements SpeedController {

    private SpeedController[] m_speedControllers;
    private double m_speed;
    private boolean m_inverted;

    public MultiSpeedController(SpeedController... speedControllers) {
        m_speedControllers = speedControllers;
        set(0);
    }

    @Override
    public double get() {
        return m_speed;
    }

    @Override
    public void set(double speed) {
        m_speed = speed;

        for (SpeedController speedController : m_speedControllers) {
            speedController.set(m_speed);
        }
    }

    @Override
    public void pidWrite(double output) {
        set(output);
    }

    @Override
    public void disable() {
        for (SpeedController speedController : m_speedControllers) {
            speedController.disable();
        }
    }

    @Override
    public boolean getInverted() {
        return m_inverted;
    }

    @Override
    public void setInverted(boolean inverted) {
        m_inverted = inverted;
        for (SpeedController speedController : m_speedControllers) {
            speedController.setInverted(m_inverted);
        }
    }

    @Override
    public void stopMotor() {
        m_speed = 0.0;

        for (SpeedController speedController : m_speedControllers) {
            speedController.stopMotor();
        }
    }
}

class HistoryWrapper {
    private History m_history = null;

    HistoryWrapper(History history) {
        m_history = history;
    }

    double getHistoryAverage(double value) {
        if (m_history == null) {
            return value;
        }
        m_history.appendToHistory(value);
        return m_history.getHistoryAverage();
    }

    double getHistoryAverageWithSalt(double value) {
        if (m_history == null) {
            return value;
        }
        return m_history.getHistoryAverageWithSalt(value);
    }
}

// This is wrapper class around RobotDrive that acts as an automatic
// transmission.
// It sets thresholds beyond which it'll shift.
// The logic hierarchy is as follows:
// 1) if we're powering at below LOW_THRESHOLD, always shift into low gear.
// 2) if we're powering above HIGH_THRESHOLD, and we have been for the last
// half-second, switch into HIGH.
class TeleopTransDrive {
    public static final int TRANS_HISTORY_LEN_MS = 250;
    public static final int SPEED_HISTORY_LEN_MS = 1000;
    public static final double HIGH_THRESHOLD = 0.80;
    public static final double LOW_THRESHOLD = 0.40;
    public static final double SLOW_MODE_CAP = 0.70;

    private ArrayList<Integer> m_buttons_forceLowTrans;
    private boolean m_forceHighTrans;
    private History m_transHistory;
    private DifferentialDrive m_drive;
    private Solenoidal m_transmission;

    public TeleopTransDrive(DifferentialDrive drive, Solenoidal transmission, int[] buttons_forceLowTrans, boolean forceHighTrans) {
        m_transmission = transmission;
        m_drive = drive;
        m_drive.setExpiration(0.1);
        m_drive.setSafetyEnabled(true);
        m_drive.setMaxOutput(1.0);
        m_buttons_forceLowTrans = new ArrayList<Integer>();
        for (int button : buttons_forceLowTrans) {
            m_buttons_forceLowTrans.add(button);
        }
        m_forceHighTrans = forceHighTrans;
        m_transHistory = new History(TRANS_HISTORY_LEN_MS, true);
    }

    public void stop() {
        m_drive.arcadeDrive(0.0, 0.0);
    }

    public void drive(Joystick stick, boolean curvature, Double abs_limit) {

        double joystick_Y = stick.getY();
        double joystick_X = stick.getX();

        // If the elevator is up, just limit our inputs.
        if (abs_limit != null) {
            joystick_Y = RobotUtils.abs_min(joystick_Y, abs_limit.doubleValue());
            joystick_X = RobotUtils.abs_min(joystick_X, abs_limit.doubleValue());
        }

        boolean forceLowTrans = false;
        for (int button : m_buttons_forceLowTrans) {
            forceLowTrans = forceLowTrans || stick.getRawButton(button);
        }

        // Actually do the Drive
        if (curvature && !forceLowTrans) {
            boolean isQuickTurn = Math.abs(stick.getX()) > 0.9  //if the joystick x (sideways) value is above this pivot turns are possible as long as the driver isn't going forward
                                  && Math.abs(stick.getY()) < 0.2; //controls how much the driver needs to go forward before curvature kicks in
            double power = Math.pow(joystick_Y, 3);
            double rotation = -joystick_X;
            m_drive.curvatureDrive(power, rotation, isQuickTurn);
        } else {
            // We're running into an issue where the turning with Neos snaps too quickly. 
            // TODO: Make this more quadratic.
            //   For now, just taper off when joystick_X is below 0.95.

            double REDUCTION_THRESHOLD_X = 1.0;
            double REDUCTION_FACTOR_X = 0.75;
            if (Math.abs(joystick_X) <= REDUCTION_THRESHOLD_X) {
                joystick_X *= REDUCTION_FACTOR_X;
            }

            m_drive.arcadeDrive(joystick_Y, -joystick_X);
        }


        // If forceHighTrans was specified
        if (m_forceHighTrans && !forceLowTrans) {
            if (highTransmission()) {
                System.out.println("Auto-Switching to High Transmission");
            }
            return;
        }

        // Enter this section only if we're actually doing auto-shifting.

        // If Auto shifting is on, we should be keeping track of the history, low or high gear.
        m_transHistory.appendToHistory(joystick_Y);

        // If the driver is holding the Low Transmission button, force it into low transmission.
        if (forceLowTrans) {
            if (lowTransmission()) {
                System.out.println("Force-Switching to Low Transmission");
            }
            return;
        }

        // If we've made it this far, we need to make a decision about 
        //   shifting given the historical joystick averages!
        double transHistoryAverage = m_transHistory.getHistoryAverage();

        // If the driver is holding the joystick below the Low Transmission
        //   threshold for long enough, downshift into Low Gear.
        if (transHistoryAverage < LOW_THRESHOLD) {
            if (lowTransmission()) {
                System.out.println("Auto-Switching to Low Transmission");
            }
            return;
        }
        // If the driver is holding the joystick above the High Transmission 
        //   threshold for long enough, upshift into High Gear.
        if (transHistoryAverage >= HIGH_THRESHOLD) {
            if (highTransmission()) {
                System.out.println("Auto-Switching to High Transmission");
            }
            return;
        }
    }

    public void arcadeDrive(Joystick stick, Double abs_limit) {
      drive(stick, false, abs_limit);
    }

    public void curvatureDrive(Joystick stick, Double abs_limit) {
      drive(stick, true, abs_limit);
    }

    public void arcadeDrive(double speed, double angle) {
        highTransmission();
        if (m_drive != null) {
            if (speed == 0.0 && angle == 0.0) {
                m_drive.stopMotor();
            } else {
                m_drive.arcadeDrive(speed, angle);
            }
        }
    }

    public boolean setTransmission(boolean target) {
        if (m_transmission == null) {
            DriverStation.reportError("Transmission called, but not instantiated\n", false);
            return false;
        }

        boolean changed = m_transmission.get() != target;
        m_transmission.set(target);
        return changed;
    }

    public boolean lowTransmission() {
        return setTransmission(false);
    }

    public boolean highTransmission() {
        return setTransmission(true);
    }
}

class LimelightDrive {
    public static final double kAIM = 0.45;
    public static final double kDistance = 5.125;
    public static final double kMinInc = 0.05;
    public static final double HATCH_PICKUP_DISTANCE = (RobotType.SOL ? 6.05 : 5.50);
    public static final double CARGO_SHOOT_DISTANCE = LimelightType.isOriginal ? 5.0 : 0.0;
    public static final double ROCKET_SHOOT_DISTANCE = LimelightType.isOriginal ? 4.5 : 0.0; 
    
    private DifferentialDrive m_drive;
    private Solenoidal m_transmission;


    public LimelightDrive(DifferentialDrive drive, Solenoidal transmission) {
        m_transmission = transmission;
        m_drive = drive;
        //m_hatchGrab = hatchGrab;

        //m_drive.setExpiration(0.1);
        //m_drive.setSafetyEnabled(true);
        //m_drive.setMaxOutput(0.5);
    }

    public void autoDrive(Limelight limelight)
    {
        // Limelight targets are updated in Robot::robotPeriodic
        double tx = limelight.get_tx();
        double ty = limelight.get_ty();
        double area = limelight.get_ta();

        double tv = limelight.get_tv();
        if (!MathUtils.fuzzyEquals(tv, 1.0)) {
            return;
        }

        // Initialize configurations to RobotMap.LimelightPipeline.HATCH values.
        double KpAim = kAIM; // SmartDashboard.getNumber("AutoDrive_kAIM", kAIM);
        double KpDistance = kDistance; // SmartDashboard.getNumber("AutoDri ve_kDistance", kDistance);
        double min_aim_command = kMinInc; // SmartDashboard.getNumber("AutoDrive_minInc", 0.05f);
        double max_area = HATCH_PICKUP_DISTANCE;
        double min_area = 0.0;

        double max_x = 23;
        double min_x = -23;

        // Flip the area; it's inverted (bigger is target originally);
        if (area != 0.0f) {
            area = max_area - area;
        }

        double max_range = (max_x + max_area);
        double min_range = (min_x + min_area);

        double range = (max_range - min_range) / 2; // 25.5
        double offset = (max_range - range); // 2.5

        double heading_error = -tx;

        double distance_error = -area; // -ty;
        double steering_adjust = 0.0f;

        if (tx > 0.0) {
            steering_adjust = KpAim * heading_error - min_aim_command;
        } else if (tx < 0.0) {
            steering_adjust = KpAim * heading_error + min_aim_command;
        }

        double distance_adjust = KpDistance * distance_error;
        SmartDashboard.putNumber("Distance_Adjust", distance_adjust);
        SmartDashboard.putNumber("Steering_Adjust", steering_adjust);

        double left_command = distance_adjust + steering_adjust;
        double right_command = distance_adjust - steering_adjust;

        double scaled_left_command = (left_command + offset) / range;
        double scaled_right_command = (right_command + offset) / range;

        SmartDashboard.putNumber("LeftCommand", scaled_left_command);
        SmartDashboard.putNumber("RightCommand", scaled_right_command);

        lowTransmission();
        m_drive.tankDrive(scaled_left_command, scaled_right_command);
    }

    public void lowTransmission() {
        if (m_transmission != null) {
            m_transmission.set(false);
        }
    }
}

class History {
    History(int historyLenMs, boolean useAbsValue) {
        m_historyLenMs = historyLenMs;
        m_transHistory = new ArrayDeque<TimeEntry>();
        m_useAbsValue = useAbsValue;
    }

    public void appendToHistory(double magnitude) {
        long now = System.currentTimeMillis();
        while (m_transHistory.peekFirst() != null && (now - m_transHistory.peekFirst().time) > m_historyLenMs) {
            m_transHistory.pollFirst();
        }
        m_transHistory.addLast(new TimeEntry(now, m_useAbsValue ? Math.abs(magnitude) : magnitude));
    }

    public double getHistoryAverage() {
        double sum = 0.0;
        if (m_transHistory.size() < 1) {
            return sum;
        }
        for (TimeEntry e : m_transHistory) {
            sum += e.magnitude;
        }

        return sum / m_transHistory.size();
    }

    public double getHistoryAverageWithSalt(double salt) {
        int size = 0;
        size = m_transHistory.size();
        return (getHistoryAverage() * size + salt) / (size + 1);
    }

    public void clear() {
        m_transHistory.clear();
    }

    private class TimeEntry {
        public TimeEntry(long time_, double magnitude_) {
            time = time_;
            magnitude = magnitude_;
        }

        public long time;
        public double magnitude;
    }

    private ArrayDeque<TimeEntry> m_transHistory;
    private int m_historyLenMs;
    private boolean m_useAbsValue;
}

class LED {
    private Solenoid m_led;

    public LED(int port) {
        m_led = new Solenoid(port);
    }

    public void on() {
        m_led.set(true);
    }

    public void off() {
        m_led.set(false);
    }
}

class MathUtils
{
    

public static boolean fuzzyEquals(double a, double b) {
    return fuzzyEquals(a, b, 0.0001);
}

public static boolean fuzzyEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
}

}
class PressureSensor {
    private final AnalogInput mAnalogInput;

    public PressureSensor(int compressorPort,int sensorAnalogInput) {
        mAnalogInput = new AnalogInput(sensorAnalogInput);
    }

    public double getAirPressurePsi() {
        // taken from the datasheet
        return 250.0 * mAnalogInput.getVoltage() / 5.0 - 25.0;
    } 
}

class Invoked {

    public Invoked() {}
    public boolean invoked;

    public boolean notInvoked(boolean invoker, boolean invokee) {

        if(invoker == true && invokee == true) {
            invoked = true;
        }
        if(invoker == false && invokee == true) {
            invoked = false;
        }
        if(invoker == true && invokee == false) {
            invoked = true;
        }
        if(invoker == false && invokee == false) {
            invoked = false;
        }
        return !invoked;
    }
}


