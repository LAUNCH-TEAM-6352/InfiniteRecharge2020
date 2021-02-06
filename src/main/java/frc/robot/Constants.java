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
		public static final int leftFrontMotorChannel = 10;
		public static final int leftRearMotorChannel = 11;
		public static final int rightFrontMotorChannel = 13;
		public static final int rightRearMotorChannel = 14;

		public static final boolean leftMotorInverted = true;
		public static final boolean rightMotorInverted = false;
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
		public static final int pipelineVision1 = 1;
		public static final int pipelineDriver2 = 2;
		public static final int pipelineVision2 = 3;
		public static final int pipelineDriver3 = 4;
		public static final int pipelineVision3 = 5;
		public static final int pipelineDefault = pipelineDriver1;

		// The desired targeting vision processing pipeline for each
		// pipeline. -1 represents no change.
		public static final int[] targetingPipelines =
		{
			pipelineVision1,
			-1,
			pipelineVision2,
			-1,
			pipelineVision3,
			-1,
			pipelineVision1,
			pipelineVision1,
			pipelineVision1,
			pipelineVision1
		};

		// The desired driving pipeline for each
		// pipeline. -1 represents no change.
		public static final int[] drivingPipelines =
		{
			-1,
			pipelineDriver1,
			-1,
			pipelineDriver2,
			-1,
			pipelineDriver3,
			pipelineDriver1,
			pipelineDriver1,
			pipelineDriver1,
			pipelineDriver1
		};
	}

	public static final class HoodMotorConstants
	{
		public static final double peakVelocityUnitsPer100ms = 153000.0;

		public static final int profileSlot = 0;
		public static final double pidP = 0.020;
		public static final double pidI = 0.0002;
		public static final double pidD = 0.0;
		public static final int pidIZ = 3000;
		public static final double pidFF = 0.0;
		public static final double pidPeakOutput = 1;
		public static final int pidLoopPeriodMs = 1;
		public static final double pidMaxIntegralAccum = 0;
		public static final int pidAllowableError = 0;
		public static final int pidTimeoutMs = 30;
		public static final double pidMinOutput = -1.0;
		public static final double idMaxOutput = 1.0;

		public static final int countsPerRevolution = 1024;
		public static final int ticksPerCount = 4;
		public static final int primaryClosedLoop = 0;

		public static final boolean phase = true;
	}

	public static final class ShooterConstants
	{
		public static final int leftMotorChannel = 22;
		public static final int rightMotorChannel = 23;
		public static final boolean isLeftMotorInverted = false;
		public static final boolean isRightMotorInverted = true;

		public static final double defaultVelocity = 2500;
		public static final double defaultPercentage = 0.5;
	}

	public static final class ShooterMotorConstants
	{
		// This is the output of the 4:1 gearbox:
		public static final double peakVelocityUnitsPer100ms = 20500.0;

		// These are for a target velocity of 2,500 RPM
		public static final double targetVelocityUnitsPer100ms = 17100.0;
		public static final double targetPercentage  = 0.72;

		public static final int channel = 9;
		public static final int profileSlot = 0;
		public static final double pidP = 0.8;
		public static final double pidI = 0.00001;
		public static final double pidD = 0.03;
		public static final int pidIZ = 3000;
		public static final double pidFF = targetPercentage * 1023.0 / targetVelocityUnitsPer100ms;
		public static final double pidPeakOutput = 1;
		public static final int pidLoopPeriodMs = 1;
		public static final double pidMaxIntegralAccum = 0;
		public static final int pidAllowableError = 0;
		public static final int pidTimeoutMs = 30;
		public static final double pidMinOutput = -1.0;
		public static final double idMaxOutput = 1.0;

		public static final int countsPerRevolution = 1024;
		public static final int ticksPerCount = 4;
		public static final int primaryClosedLoop = 0;

		public static final boolean phase = false;
	}

	public static final class TurretConstants
	{
		public static final int hoodMotorChannel = 20;
		public static final int azimuthMotorChannel = 21;

		public static final int upHoodLimitChannel = 8;
		public static final int downHoodLimitChannel = 9;

		public static final int leftAzimuthLimitChannel = 5; 
		public static final int centerAzimuthLimitChannel = 6; 
		public static final int rightAzimuthLimitChannel = 7;
		
		public static final boolean isAzimuthMotorInverted = true;
		public static final boolean isHoodMotorInverted = true;

		public static final double hoodUpPercentageScaleFactor = 1.0;
		public static final double hoodDownPercentageScaleFactor = 1.0;
		public static final double azimuthPercentageScaleFactor = 1.00;

		public static final double moveToLeftAutoPercentage = -1.0;
		public final static double moveToRightAutoPercentage = +1.0;

		public final static double moveHoodDownAutoPercentage = -1.0;
		public final static double moveHoodUpAutoPercentage = +1.0;

		public final static int hoodDownPosition = 0;
		public final static int hoodupPosition = 1439854;

		public final static int moveHoodToPositionTolerance = 1000;
		public final static int moveHoodToPositionTimeoutInSeconds = 3;

		public static final int defaultHoodTargetPosition = 700000;
	}

	public static final class TargetingConstants
	{
		public static final double maxAzimuthSpeed = 1.0;
		public static final double minAzimuthSpeed = 0.1;
		public static final double maxAzimuthDelta = 30.0;
		public static final double minAzimuthDelta = 0.2;
		public static final double azimuthTolerance = 0.2;
	}


	public static final class IntakeConstants
	{
		public static final int motorChannel = 41;
		public static final boolean isIntakeMotorInverted = false;

		public static final int pcmChannel = 0;
		public static final int inSolenoidChannel = 0;
		public static final int outSolenoidChannel = 1;

		public static final double defaultInSpeed = 0.90;
		public static final double defaultOutSpeed = -0.90;
	}

	public static final class IndexerConstants
	{
		public static final int rearMotorChannel = 51;
		public static final boolean isRearMotorInverted = true;
		public static final double defaultForwardSpeed = 0.60;
		public static final double defaultReverseSpeed = -0.60;
	}

	public static final class ClimberConstants
	{
		public static final int leftMotorChannel = 31;
		public static final int rightMotorChannel = 32;

		public static final int outSolenoidChannel = 2;
		public static final int inSolenoidChannel = 3;

		public static final boolean isLeftMotorInverted = false;
		public static final boolean isRightMotorInverted = false;

		public static final int leftRetractedLimit = 1;
		public static final int leftExtendedLimit = 2;
		public static final int rightRetractedLimit = 3;
		public static final int rightExtendedLimit = 4;

		public static final double extendSpeed = -0.5;
	}

	public static final class OIConstants
	{
		public static final int xboxControllerPort = 1;
		public static final int leftJoystickPort = 2;
		public static final int rightJoystickPort = 3;
	}

	public static final class DashboardConstants
	{
		public static final String setVelocityKey = "Set Velocity";
		public static final String downHoodLimitKey = "Hood Down";
		public static final String upHoodLimitKey = "Hood Up";
		public static final String azimuthMotorKey = "Azimuth Motor";
		public static final String leftAzimuthLimitKey = "Azimuth Left";
		public static final String frontAzimuthLimitKey = "Azimuth Front";
		public static final String rightAzimuthLimitKey = "Azimuth Right";
		public static final String turretPositionKey = "Turret Pos";

		public static final String shooterTargetVelocityKey = "Shooter Vel Tgt";
		public static final String shooterSetVelocityKey = "Shooter Vel Set";
		public static final String shooterCurrentVelocityLeftKey = "Shooter Vel Left";
		public static final String shooterCurrentVelocityRightKey = "Shooter Vel Rght";

		public static final String shooterTargetPercentageKey = "Shooter %";

		public static final String hoodTargetPositionKey = "Hood Target Pos";
		public static final String hoodCurrentPositionKey = "Hood Current Pos";

		public static final String indexerForwardPercentageKey = "Indexer Fwd %";
		public static final String indexerReversePercentageKey = "Indexer Rev %";

		public static final String intakeInPercentageKey = "Intake In %";
		public static final String intakeOutPercentageKey = "Intake Out %";

		public static final String hoodMotorKey = "Hood Motor";

		public static final String leftClimberExtendedLimitKey = "L Climb Ext";
		public static final String leftClimberRetractedLimitKey = "L Climb Ret";
		public static final String rightClimberExtendedLimitKey = "R Climb Ext";
		public static final String rightClimberRetractedLimitKey = "R Climb Ext";

		public static final String limelightTaKey = "Limelight ta";
		public static final String limelightTxKey = "Limelight tx";
		public static final String limelightTyKey = "Limelight ty";
	}
}
