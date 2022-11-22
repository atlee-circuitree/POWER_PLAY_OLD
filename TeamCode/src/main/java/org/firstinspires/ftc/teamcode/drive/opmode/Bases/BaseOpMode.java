package org.firstinspires.ftc.teamcode.drive.opmode.Bases;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.opmode.Bases.kauailabs.navx.ftc.AHRS;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

@Config
public abstract class BaseOpMode extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor frontLeft = null;
    public DcMotor rearLeft = null;
    public DcMotor frontRight = null;
    public DcMotor rearRight = null;
    public DcMotorEx vertArm = null;
    public DcMotorEx horizArm = null;
    public DcMotorEx angleArm = null;

    public Servo horizClaw = null;
    public Servo transferClaw = null;
    public Servo transferArmTop = null;
    public Servo transferArmBotttom = null;

    /*public DistanceSensor LS_distance;
    public DistanceSensor RS_distance;
    public DistanceSensor RL_distance;
    public DistanceSensor RR_distance;*/

    public double SD = 1;
    public double SA = 1;

    public PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double HORIZONTAL_CLAW_OPEN = .5;
    public static double HORIZONTAL_CLAW_CLOSE = .6;
    public static double TRANSFER_CLAW_OPEN = .8;
    public static double TRANSFER_CLAW_CLOSE = .75;

    public static double TRANSFER_ARM_TOP_FRONT = .45;
    public static double TRANSFER_ARM_TOP_BACK = .2;
    public static double TRANSFER_ARM_BOTTOM_FRONT = .44;
    public static double TRANSFER_ARM_BOTTOM_BACK = .66;

    public static double armLengthWorm;
    public static double armHeightWorm;
    public static double armLengthClaw;

    public static int horizArmTarget = 0;
    public final double ticks_in_degrees = 384.5; //Arm motor ticks

    public final static double ARM_DEFAULT = 0.5; //Unslash this if you want armTurn servo using joystick back (This is for variable turn of a servo)
    public final static double ARM_MIN_RANGE = 0.46;
    public final static double ARM_MAX_RANGE = 0.53;

    public AHRS navx_centered;

    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;

    //turn motor at 200 ticks per second
    public double motorVelocity = 200;

    static final double     COUNTS_PER_MOTOR_REV    = 383.6;    // eg: GOBUILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
    public static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE           = 1;

    static final double     OMNI_COUNTS_PER_REV     = 8192; //For rev through bore encoder (This is the correct number)
    static final double     OMNI_WHEEL_DIAMETER     = 3.77953;
    public static final double      OMNI_COUNTS_PER_INCH    = (OMNI_COUNTS_PER_REV) / (OMNI_WHEEL_DIAMETER * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    //Initializes hardware
    public void GetHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        getCenteredNavXValues();
        //Motor and Servo Variables
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        horizArm = hardwareMap.get(DcMotorEx.class, "horizArm");
        vertArm = hardwareMap.get(DcMotorEx.class, "vertArm");
        angleArm = hardwareMap.get(DcMotorEx.class, "angleArm");

        horizClaw = hardwareMap.get(Servo.class, "horizClaw");
        transferClaw = hardwareMap.get(Servo.class, "transferClaw");
        transferArmTop = hardwareMap.get(Servo.class, "transferArmTop");
        transferArmBotttom = hardwareMap.get(Servo.class, "transferArmBottom");

        /*LS_distance = hardwareMap.get(DistanceSensor.class, "LS_distance");
        RS_distance = hardwareMap.get(DistanceSensor.class, "RS_distance");
        RL_distance = hardwareMap.get(DistanceSensor.class, "RL_distance");
        RR_distance = hardwareMap.get(DistanceSensor.class, "RR_distance");*/

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        vertArm.setDirection(DcMotor.Direction.REVERSE);

        SetDriveMode(Mode.STOP_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vertArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angleArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        horizArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SetDriveMode(Mode.RUN_WITH_ENCODER);
    }

    public void GetIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("IMU", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("IMU", "calibrated");
        telemetry.update();
    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360 * 2;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;

    }

    /*public void calculateHorizHeight(int armLengthWorm, int armHeightWorm, int arm) {
        final int
        int

        int armLengthFull = armLengthClaw(armHeightWorm/armLengthWorm)
    }*/

    public void angleArmPIDLoop() {
        controller.setPID(p, i, d);
        int horizArmPos = horizArm.getCurrentPosition();
        double pid = controller.calculate((horizArmPos), horizArmTarget);
        double ff = Math.cos(Math.toRadians(horizArmTarget / ticks_in_degrees)) * f;

        double horizArmPower = pid + ff;

        horizArm.setPower(horizArmPower);

        telemetry.addData("Horiz Arm Pos", horizArmPos);
        telemetry.addData("Horiz Arm Target", horizArmTarget);
    }



/*
    public void getGyro() {

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }
    }
    */

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     FLTarget;
        int     FRTarget;
        int     RLTarget;
        int     RRTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  FLSpeed;
        double  RLSpeed;
        double  FRSpeed;
        double  RRSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);

            FLTarget = frontLeft.getCurrentPosition() + moveCounts;
            RLTarget = rearLeft.getCurrentPosition() + moveCounts;
            FRTarget = frontRight.getCurrentPosition() + moveCounts;
            RRTarget = rearRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontLeft.setTargetPosition(FLTarget);
            frontRight.setTargetPosition(FRTarget);
            rearLeft.setTargetPosition(RLTarget);
            rearRight.setTargetPosition(RRTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            rearLeft.setPower(speed);
            rearRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                FLSpeed = speed - steer;
                RLSpeed = speed - steer;
                FRSpeed = speed + steer;
                RRSpeed = speed + steer;

               // max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));


                // Normalize speeds if either one exceeds +/- 1.0;

               // max = Math.max(Math.abs(FLSpeed), Math.abs(FRSpeed), Math.max(Math.abs(RRSpeed), Math.abs(RLSpeed));

                //max = Math.max(Math.abs(FLSpeed), (RLSpeed), (Math.abs(FRSpeed), (RRSpeed));

               // max = Math.max(Math.abs(FLSpeed), Math.abs(FRSpeed));
                max = Math.max(Math.abs(RRSpeed), Math.abs(RLSpeed));

                if (max > 1.0)
                {
                    FLSpeed /= max;
                    FRSpeed /= max;
                    RRSpeed /= max;
                    RLSpeed /= max;
                }
                frontLeft.setPower(FLSpeed);
                frontRight.setPower(FRSpeed);
                rearLeft.setPower(RLSpeed);
                rearRight.setPower(RRSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      FLTarget,  FRTarget, RLTarget, RRTarget);
                telemetry.addData("Actual",  "%7d:%7d",      frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  FLSpeed, FRSpeed, RLSpeed, RRSpeed);
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double  FLSpeed;
        double  RLSpeed;
        double  FRSpeed;
        double  RRSpeed;
        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            FLSpeed  = 0.0;
            RLSpeed = 0.0;
            FRSpeed  = 0.0;
            RRSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            FRSpeed = speed * steer;
            RRSpeed  = speed * steer;
            FLSpeed = -FRSpeed;
            RLSpeed   = -RRSpeed;

        }

        // Send desired speeds to motors.
        frontLeft.setPower(FLSpeed);
        frontRight.setPower(FRSpeed);
        rearLeft.setPower(RLSpeed);
        rearRight.setPower(RRSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", FLSpeed, FRSpeed, RLSpeed, RRSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    public enum STRAFE {
        LEFT, RIGHT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }
    public enum Drive {
        STOP
    }
    public enum Shoot{
        SHOOT_FAR,
        GET_VELOCITY,
    }

    public void DriveTrain (Drive Stop){
        if (Stop == Drive.STOP) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        }
    }

    public void encoderLift(double speed, double timeoutS) {
        int newLiftTarget;
    }

    public void SetDriveMode(Mode DriveMode) {

        if (DriveMode == Mode.STOP_RESET_ENCODER) {

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        if (DriveMode == Mode.RUN_WITH_ENCODER) {

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (DriveMode == Mode.RUN_WITHOUT_ENCODERS) {

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    public void getCenteredNavXValues(){

        navx_centered = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx_centered"), AHRS.DeviceDataType.kProcessedData);
        //boolean connected = navx_device.isConnected();
        //telemetry.addData("1 navX-Device", connected ?
        //"Connected" : "Disconnected" );
        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");

        yaw = df.format(navx_centered.getYaw());
        pitch = df.format(navx_centered.getPitch());
        roll = df.format(navx_centered.getRoll());
        ypr = yaw + ", " + pitch + ", " + roll;


        telemetry.addData("Yaw, Pitch, Roll", ypr);

    }
    public void zeroGyro() {
        navx_centered.zeroYaw();
    }


}





