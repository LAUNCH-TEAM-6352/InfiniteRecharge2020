/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase
{
	private final VictorSPX leftMotor1;
	private final VictorSPX leftMotor2;
	private final VictorSPX rightMotor1;
	private final VictorSPX rightMotor2;

	/**
	 * Creates a new DriveTrain.
	 */
	public DriveTrain()
	{
		leftMotor1 = new VictorSPX(DriveTrainConstants.LeftMotor1Channel);
		leftMotor2 = new VictorSPX(DriveTrainConstants.LeftMotor2Channel);
		rightMotor1 = new VictorSPX(DriveTrainConstants.RightMotor1Channel);
		rightMotor2 = new VictorSPX(DriveTrainConstants.RightMotor2Channel);

		leftMotor1.setInverted(DriveTrainConstants.LeftMotorInverted);
		rightMotor1.setInverted(DriveTrainConstants.RightMotorInverted);

		leftMotor2.setInverted(InvertType.FollowMaster);
		rightMotor2.setInverted(InvertType.FollowMaster);

		leftMotor2.set(ControlMode.Follower, leftMotor1.getDeviceID());
		rightMotor2.set(ControlMode.Follower, rightMotor1.getDeviceID());
	}

	/**
	 * Team Caution style drive using input from two joysticks, left and right.
	 * @param leftStick
	 * @param rightStick
	 */
	public void driveCaution(Joystick leftStick, Joystick rightStick)
	{
		setLeftRightMotorOutputs(leftStick.getY() - rightStick.getX(), leftStick.getY() + rightStick.getX());
	}

	/**
	 * Squares inputs for more sensitivity.
	 * 
	 * @param left
	 * @param right
	 */
	public void setLeftRightMotorOutputs(double left, double right)
	{
		left = MathUtil.clamp(left, -1.0, +1.0);
		right = MathUtil.clamp(right, -1.0, +1.0);
		
		leftMotor1.set(ControlMode.PercentOutput, Math.copySign(left * left, left));
		rightMotor1.set(ControlMode.PercentOutput, Math.copySign(right * right, right));
	}

	/**
	 * Stop the drive;
	 */
	public void stop()
	{
		leftMotor1.set(ControlMode.PercentOutput, 0);
		rightMotor1.set(ControlMode.PercentOutput, 0);
	}

	@Override
	public void periodic()
	{
		// This method will be called once per scheduler run
	}
}
