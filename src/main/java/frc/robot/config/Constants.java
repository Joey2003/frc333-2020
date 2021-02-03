  package frc.robot.config;


/**
 * Add your docs here.
 */
public class Constants {

    /* real numbers from robot */
    public static final double WHEEL_DIAMETER   = 5.0;   //(inches)
    public static final double GEAR_RATIO       = 0.00;
    public static final double MAX_SPEED_HIGH   = 17.0;  //(ft/sec)
    public static final double MAX_SPEED_LOW    = 4.0;   //(ft/sec)
    public static final double MAX_ACCELERATION = 0.0;


    /* values from the Robot Characterization Tool */
    public static final double ks           = 0.0;
    public static final double kV           = 0.0;
    public static final double kA           = 0.0;
    public static final double TRACK_WIDTH  = 0.0;

    //TODO: CHANGE THESE POSITION VALUES
    public static final double INIT_LINE_POSITION      = 0.0;  // Initial point
    public static final double TARGET_ZONE_POSTION     = 0.2;  // OTHER MIDDLE (naht middle)
    public static final double FAR_TRENCH_POSTITION    = 0.3;  // BIG MIDDLE (actually middle)
    public static final double NEAR_TRENCH_POSTION     = 0.55; // TOP (dont go past this point)

    /* Hood Motor Values */
    public static final int kHoodSlotIdx          = 0;
    public static final int kHoodPIDLoopIdx       = 0;
    public static final int kHoodTimeoutMs        = 30;
    public static boolean kHoodSensorPhase        = true;
    public static boolean kHoodMotorInvert        = false;
    public static final Gains kHoodGains          = new Gains(0.00005, 0, 0, 0, 0.0003, 1, -1); // May need to be tuned
    public static final double kHoodCuspPower     = .2;
    public static final double kHoodmaxVel        = 14000;
    public static final double kHoodmaxAcc        = 14000;
    public static final double kHoodminVel        = 0;
    public static final double kHoodAllowedErr    = 0;
    public static final int kHoodSmartMotionSlot  = 0;
    public static final int kHoodCPR              = 4096; // Mag Encoder Counts Per Revolution



      /* Turret Motor Values */
      public static final int kTurretSlotIdx          = 0;
      public static final int kTurretPIDLoopIdx       = 0;
      public static final int kTurretTimeoutMs        = 30;
      public static boolean kTurretSensorPhase        = true;
      public static boolean kTurretMotorInvert        = false;
      public static final Gains kTurretGains          = new Gains(0.00002, 0, 0, 0, 0.0003, .6, -.6); // May need to be tuned
      public static final double kTurretmaxVel        = 60;
      public static final double kTurretmaxAcc        = 10;
      public static final double kTurretminVel        = 0;
      public static final double kTurretAllowedErr    = 20;
      public static final int kTurretSmartMotionSlot  = 0;
      public static final int kTurretCPR              = 4096; // Mag Encoder Counts Per Revolution
      public static final double kControllerDeadband     = 0.0;


    /* Shooter Constants */
    public static final double kShooterFeederPower  = 0.5;
    public static final double kShooterEpsilon      = 20.0;
    public static final double kShooterRPM          = 1000.0;
    public static final Gains kShooterGains         = new Gains(0.00005, 0, 0, 0, 0.0002, 1, 0); // May need to be tuned
    public static final boolean kShooterDebug       = false;
    public static final double kShooterAcceleration = kShooterRPM * .5;

    /* hopper/roller Constants */
    public static final double  kRollerSpeed        = 0.5;
    public static final double  kCyclerSpeed        = 1.0;
    public static final double  kConveyerSpeed      = -0.8;
    public static final double  kthroatSpeed        = 0.8;




}