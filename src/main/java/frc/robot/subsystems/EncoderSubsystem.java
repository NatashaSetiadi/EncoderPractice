/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commands.ElevatorPositionCommand;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Add your docs here.
 */
public class EncoderSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorPositionCommand());
  }

  // create the parts of the swerve that we can actually control [Motors,
  // Encoders, etc]
  private WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(1);
  private Encoder encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private double steerOffset = Math.toDegrees(100);

  private final double Kp = 1;
  private final double Ki = 0;
  private final double Kd = 0;

  PIDController pid = new PIDController(Kp, Ki, Kd);

  public EncoderSubsystem() {
    pid.enableContinuousInput(-180, 180);
  }

  public void motorSet() {
    elevatorMotor.set(MathUtil.clamp(pid.calculate(encoder.getDistance(), 180), -0.5, 0.5));
    System.out.println("PID Output:" + MathUtil.clamp(pid.calculate(encoder.getDistance(), 180), -0.5, 0.5));
  }

  public void set() {

  }

  public void stop() {

  }
}
