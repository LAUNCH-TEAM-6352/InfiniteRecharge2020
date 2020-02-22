/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static final class DriveTrainConstants
	{
		public static final int LeftMotor1Channel = 10;
		public static final int LeftMotor2Channel = 11;
		public static final int RightMotor1Channel = 13;
		public static final int RightMotor2Channel = 14;

		public static final boolean LeftMotorInverted = true;
		public static final boolean RightMotorInverted = true;
	}

	public static final class LimelightConstants
	{
		public static final String tableName = "limelight";
		public static final String pipelineEntryName = "pipeline";
		public static final String xPositionEntryName = "tx";
		public static final String yPositionEntryName = "ty";
		public static final String targetAcquiredEntryName = "tv";
		public static final String areaEntryName = "ta";
		public static final String camModeEntryName = "camMode";
		public static final String ledModeEntryName = "ledMode";

		public static final int ledModePipeline = 0;
		public static final int ledModeOff = 1;
		public static final int ledModeOn = 3;

		public static final int pipelineDriver1 = 0;
		public static final int pipelineDriver2 = 1;
		public static final int pipelineDriver3 = 2;
		public static final int pipelineVision1 = 3;
		public static final int pipelineVision2 = 4;
		public static final int pipelineVision3 = 5;
		public static final int pipelineDefault = pipelineDriver1;

		// The desired targeting vision processing pipeline for each
		// pipeline. -1 represents no change.
		public static final int[] targetingPipelines =
		{
			pipelineVision1,
			pipelineVision2,
			pipelineVision3,
			-1,
			-1,
			-1,
			pipelineVision1,
			pipelineVision1,
			pipelineVision1,
			pipelineVision1
		};
	}

}
