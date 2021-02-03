/**
 *  Class that organizes gains used when assigning values to slots
 * 
 *  >>Used in Constants class for hood
 */
package frc.robot.config;

public class Gains {
	//TODO: move to robotUtils!
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kFF;
	public final double kIz;
	public final double kMaxOutput;
	public final double kMinOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kIz, double _kFF, double _kMaxOutput, double _kMinOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kFF = _kFF;
		kIz = _kIz;
		kMaxOutput = _kMaxOutput;
		kMinOutput = _kMinOutput;
	}
}