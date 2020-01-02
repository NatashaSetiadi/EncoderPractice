/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int LEFT_FRONT_DRIVE_TALON_PORT = 6;
	public static final int LEFT_BACK_DRIVE_TALON_PORT = 7;
	public static final int RIGHT_FRONT_DRIVE_TALON_PORT = 1;
	public static final int RIGHT_BACK_DRIVE_TALON_PORT = 3;
	public static final int GRABBER_TALON_PORT = 8;
	public static final int XBOX_CONTROLLER_1_PORT = 0;
	public static final int XBOX_CONTROLLER_2_PORT = 1;

	public final static double deadzone = .18;
	public final static double TRACKWIDTH = 30; // distance from the centers of the wheels width wise
	public final static double WHEELBASE = 30; // distance between the centers of the wheels length wise

	// save vector length (magnitude of the vector of the chassis) to a double value
	public final static Vector2d chassisVector = new Vector2d(TRACKWIDTH, WHEELBASE);
	public final static double chassisMagnitude = chassisVector.magnitude();

}
