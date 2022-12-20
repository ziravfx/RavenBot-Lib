package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.dependencies.CMYSleeveDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class robot {
	private LinearOpMode myOpMode;

	private DcMotor FL = null;
	private DcMotor BL = null;
	private DcMotor BR = null;
	private DcMotor FR = null;
	private DcMotor arm = null;
	private OpenCvCamera camera;
	private IMU imu;

	public CMYSleeveDetection sleeveDetection;

	public robot (LinearOpMode opmode) {
		myOpMode = opmode;
	}

	/**initializing all our hardware, different modules for different aspects*/
	public void init(){
		FL = myOpMode.hardwareMap.dcMotor.get("motorFrontLeft");
		BL = myOpMode.hardwareMap.dcMotor.get("motorBackLeft");
		BR = myOpMode.hardwareMap.dcMotor.get("motorBackRight");
		FR = myOpMode.hardwareMap.dcMotor.get("motorFrontRight");
		arm = myOpMode.hardwareMap.dcMotor.get("armMotor");

		//tells the arm to hold its position and actively resist moving when no power is applied
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//reverse for funsies
		BL.setDirection(DcMotorSimple.Direction.REVERSE);
		FL.setDirection(DcMotorSimple.Direction.REVERSE);
		//BR.setDirection(DcMotorSimple.Direction.REVERSE);
		//FR.setDirection(DcMotorSimple.Direction.REVERSE);

		FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

	}
	public void OCVinit(){
		String webcamName = "Webcam 1";
		int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
		sleeveDetection = new CMYSleeveDetection();
		camera.setPipeline(sleeveDetection);

		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
			}

			@Override
			public void onError(int errorCode) {}
		});
	}
	public void IMUinit (){
		imu = myOpMode.hardwareMap.get(IMU.class, "imu");
		IMU.Parameters IMUparams;//logo BACK, USB UP
		IMUparams = new IMU.Parameters(
			new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
					RevHubOrientationOnRobot.UsbFacingDirection.UP
			)
		);
		imu.initialize(IMUparams);
		imu.resetYaw();
	}

	/**variables and referentials*/
	private final int motorTickCount = 763;
	private final int armTickCount = 560;
	private final float wheelCircumference = 3.77953f;
	private final float armCircumference = 1.4f;
	private YawPitchRollAngles robotOrient;
	/**data manipulation methods*/
	private int ticksNeeded(int tickCount, double wheelC, float dist){
		double circum = 3.141592 * wheelC;
		double rotNeeded = dist/circum;
		int tNeeded = (int) rotNeeded*tickCount;
		return tNeeded;
	}
	/**time-savers that control multiple motors at once*/
	public void setMotorPower(double FLP, double BLP, double BRP, double FRP){
		FL.setPower(FLP);
		BL.setPower(BLP);
		BR.setPower(BRP);
		FR.setPower(FRP);
	}
	public void setAllPower(double Pwr){
		setMotorPower(Pwr, Pwr, Pwr, Pwr);
	}
	private void stopAndReset(){
		FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}
	private void setEncoderTarget(int FLE, int BRE, int BLE, int FRE){
		FL.setTargetPosition(FLE);
		FR.setTargetPosition(FRE);
		BL.setTargetPosition(BLE);
		BR.setTargetPosition(BRE);
	}
	private void runToPos(){
		FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
	/**slows down the motors as we get closer to the target position*/
	private void gradPwr(DcMotor motor, int target, double Pwr){
		int currPos = motor.getCurrentPosition();
		float delta = (target-currPos);
		motor.setPower(Pwr - 1/delta);
	}

	/**actual movement methods, derived from master "move" method */
	public void move(double Pwr, float FLD, float BRD, float BLD, float FRD) {
		stopAndReset();
		int FLE = ticksNeeded(motorTickCount, wheelCircumference, FLD);
		int BRE = ticksNeeded(motorTickCount, wheelCircumference, BRD);
		int BLE = ticksNeeded(motorTickCount, wheelCircumference, BLD);
		int FRE = ticksNeeded(motorTickCount, wheelCircumference, FRD);
		setEncoderTarget(FLE, BRE, BLE, FRE);
		setAllPower(Pwr);
		runToPos();
		while (FR.isBusy() || BR.isBusy() || BL.isBusy() || FL.isBusy()) {
			gradPwr(FR, FRE, Pwr);
			gradPwr(FL, FLE, Pwr);
			gradPwr(BR, BRE, Pwr);
			gradPwr(BL, BLE, Pwr);
			//slows down motor as it gets closer to target
		}
	}
	/**Directional methods*/
	public void forward(double Pwr, float Dist){
		move(Pwr, Dist, Dist, Dist, Dist);
	}
	public void back(double Pwr, float Dist){
		forward(Pwr, -Dist);
	}
	/**strafing methods*/
	public void right(double Pwr, float Dist){
		move(Pwr, Dist, Dist, -Dist, -Dist);
	}
	public void left(double Pwr, float Dist){
		move(Pwr, -Dist, -Dist, Dist, Dist);
	}

	/**teleop driving*/
	public void drive(double Drive, double Strafe, double Turn){
		double denominator = Math.max(Math.abs(Drive) + Math.abs(Strafe) + Math.abs(Turn), 1.75);
		double flpwr = Drive+Strafe+Turn / denominator;
		double frpwr = Drive-Strafe-Turn / denominator;
		double brpwr = Drive+Strafe-Turn / denominator;
		double blpwr = Drive-Strafe+Turn / denominator;
		setMotorPower(flpwr, blpwr, brpwr, frpwr);
	}

 	public double yaw(){
		robotOrient = imu.getRobotYawPitchRollAngles();
		return robotOrient.getYaw(AngleUnit.DEGREES);
	}
	/**returns the robot Z axis rotation, or Heading/Yaw. New changes in 8.1 SDK make this independent of control hub orientation.**/
	public enum sideOfTurn{
		NONE,
		ACC,
		CC
	}
	public sideOfTurn getSideofTurn(double deg){
		sideOfTurn side = sideOfTurn.NONE;
		if(yaw() > deg){
			side = sideOfTurn.ACC;
		} else if(yaw() < deg){
			side = sideOfTurn.CC;
		} else if(yaw() == deg){
			side = sideOfTurn.NONE;
		}
		return side;
	}

	private boolean opmodeRunning(LinearOpMode opmode){
		return opmode.opModeIsActive() && opmode.isStopRequested();
	}

	public void absTurn(double Pwr, double Deg){
		sideOfTurn turnSide = getSideofTurn(Deg);
		double currAngle = yaw();
		switch (turnSide){
			case NONE:
				break;
			case ACC:
				while(currAngle != Deg && opmodeRunning(myOpMode)){
					currAngle = yaw();
					setMotorPower(-Pwr, -Pwr, Pwr, Pwr);
				}
				break;
			case CC:
				while(currAngle != Deg && opmodeRunning(myOpMode)){
					currAngle = yaw();
					setMotorPower(Pwr, Pwr, -Pwr, -Pwr);
				}
				break;
		}
		setAllPower(0);

	}
	public void turn(double Pwr, double Deg){
		double degtarget = Deg + yaw();
		absTurn(Pwr, degtarget);
	}

	/**
	Breakpoint! All the complicated stuff is now over :)
	Arm code begins here :)
	**/

	public void armDrive(double Drive){
		arm.setPower(Drive);
	}
	public void armTurn(float Dist){
		arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm.setTargetPosition(ticksNeeded(armTickCount, armCircumference, Dist));
		arm.setPower(.75);
		arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		while(arm.isBusy()){
			gradPwr(arm, ticksNeeded(armTickCount, armCircumference, Dist), Dist);
		}
		arm.setPower(0);
	}
	private static final float armOffset = 2.5f;
	/**accounts for the fact that the arm is off the ground. *ALL UNITS ARE IN INCHES!* */
	private enum armHeights{
		SHORT(13.5f- armOffset),
		MEDIUM(26.5f- armOffset),
		TALL(33.5f- armOffset);
		private float height;
		public float height(){
			return this.height;
		}
		private armHeights(float height){
			this.height	= height;
		}
	}
	public void armScore(armHeights heights){
		switch (heights){
			case SHORT:
				armTurn(armHeights.SHORT.height());
				break;
			case MEDIUM:
				armTurn(armHeights.MEDIUM.height());
				break;
			case TALL:
				armTurn(armHeights.TALL.height());
				break;
		}

	}
	/**
	CMYSleeveDetection method passthrough to incorporate everything comprehensively
	 */
	public CMYSleeveDetection.ParkingPosition getParkPos(){
		return sleeveDetection.getPosition();
	}

}

