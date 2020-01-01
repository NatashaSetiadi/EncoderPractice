package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commands.ElevatorPositionCommand;

public class EncoderSubsystem{

    // create the parts of the swerve that we can actually control [Motors,
    // Encoders, etc]
    private WPI_TalonSRX elevatorMotor;
    private Encoder encoder = new Encoder(0, 1, false,Encoder.EncodingType.k4X);
    private double steerOffset;

    private final double Kp = 1;
    private final double Ki = 0;
    private final double Kd = 0;

    PIDController pid = new PIDController(Kp, Ki, Kd);

    public EncoderSubsystem(){}
    public EncoderSubsystem(int elevatorMotor, double steerOffset){
        this.elevatorMotor = new WPI_TalonSRX(elevatorMotor);
        //setup steeroffset and convert it from degrees to radians
        this.steerOffset = Math.toDegrees(steerOffset);

        //enable continuous input from 0 to 2 pi radians
        pid.enableContinuousInput(-180, 180);

    }
    @Override
	public void initDefaultCommand() {
        this.setDefaultCommand(new ElevatorPositionCommand());    
    }
    public void motorSet(){
        elevatorMotor.set(MathUtil.clamp(pid.calculate(encoder.getDistance(), 180), -0.5, 0.5));
        System.out.println("PID Output:" + MathUtil.clamp(pid.calculate(encoder.getDistance(), 180), -0.5, 0.5));
    }
  
    public void set() {
		
	}


	public void stop() {
		
	}
    
}