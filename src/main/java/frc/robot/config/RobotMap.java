package frc.robot.config;

public class RobotMap { 
    public static class PlayerButton {
        public static final int INTAKE                      = 3;
        public static final int[] FORCE_LOW_TRANSMISSION    = { 2 };
        public static final int INTAKE_LIMELIGHT_DRIVE      = -3;
        public static final int TURRET_TRACK                = 9;

        public static final int INIT_LINE                   = 1;  
        public static final int TARGET_ZONE                 = 2;     
        public static final int FAR_TRENCH                  = 0;
        public static final int NEAR_TRENCH                 = 3;
        public static final int RECORD                      = 8; //SHOULD BE CHANGED TO -1 AFTER RECORDING IS DONE!!!

    }

    public static class DefensePlayerButton {}

    public static class DigitalInputPort {}
 
    public static class AnalogPort {}

    public static class TalonPort {
        public static final int GIVER_MOTOR       =  0;
        public static final int TURRET_MOTOR      =  15;
    }
    
    public static class CANSparkID {
        public static final int LEFT_LEADER       =  1;
        public static final int LEFT_FOLLOWER     =  2;
        public static final int RIGHT_LEADER      =  5;
        public static final int RIGHT_FOLLOWER    =  6;
        public static final int SHOOTER_LEFT      =  12;
        public static final int SHOOTER_RIGHT     =  13;
        public static final int HOOD_MOTOR        =  11;
        public static final int THROAT_MOTOR      =  10; 
        public static final int CYCLER_MOTOR      =  8; 
        public static final int CONVEYER_MOTOR    =  9;
        public static final int CLIMBER_MOTOR     =  14;
        public static final int INTAKE_MOTOR      =  7;
    }

    public static class Sensor_Port {
        public static final int SHOOTERENCODER = 0;
        public static final int PRESSURESENSOR = 0;
    }

    public static class SolenoidPort {
        public static final int DRIVE_TRANS_1     = 1;
        public static final int DRIVE_TRANS_2     = 2;
        public static final int ROLLER_IN         = 3;
        public static final int ROLLER_OUT        = 4;
    }

    public static class CompressorPort {
        public static final int MAIN_COMPRESSOR   = -1;
    }

    public static class ControllerPort {
        public static final int LIGHTSABER = 0;
        public static final int GUNNER     = 1;
    }
    
   public static enum AutoMode {
        TEST, MOVE_FORWARD, MOVE_BACKWARD
    };

    public static class RobotType{
        public static final boolean SOL = false;
    }

    public static class LimelightConservativeLED {
        public static final boolean isDoomsday = true;
    }

    public static class LimelightType{
        public static final boolean isOriginal = true;
    }

    public static class ControlType{
        public static final boolean controller = false;
    }

    public static class LimelightPipeline {
        public static final int POWER_PORT = 0;
        public static final int PICKUP_POINT = 1;
    }

    public static class LimelightLEDMode {
        public static final int PIPELINE = 0;
        public static final int OFF      = 1;
        public static final int BLINK    = 2;
        public static final int ON       = 3;
    }
}