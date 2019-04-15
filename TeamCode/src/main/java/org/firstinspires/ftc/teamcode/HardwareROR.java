package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static java.lang.Thread.sleep;

public class HardwareROR {

    private Telemetry hwTelemetry;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 28, DRIVE_GEAR_REDUCTION = 19.8, WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI /*3.1415*/);

    LinearOpMode opmode;

    public boolean testMode;

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    public DcMotor hangLift, intakeLift, scoringLift, sweeper;
    public Servo gate, scoringBucket;

    public Servo intakeRotateLeft, intakeRotateRight;
    public RevBlinkinLedDriver blinkin;

    public BNO055IMU navigationImu = null;

    public Orientation navigationAngles;

    HardwareMap hwMap = null;

    public HardwareROR() {

    }

    public void init(HardwareMap ahwMap, LinearOpMode opMode, Telemetry telemetry, boolean testMode, boolean resetEncoders) {
        hwMap = ahwMap;

        this.opmode = opMode;
        this.testMode = testMode;

        hwTelemetry = telemetry;

        frontLeftDrive = hwMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hwMap.get(DcMotor.class, "backLeft");
        backRightDrive = hwMap.get(DcMotor.class, "backRight");

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (resetEncoders) {
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangLift = hwMap.get(DcMotor.class, "hangLift");
        intakeLift = hwMap.get(DcMotor.class, "intakeLift");
        scoringLift = hwMap.get(DcMotor.class, "scoringLift");
        sweeper = hwMap.get(DcMotor.class, "sweeper");

        gate = hwMap.get(Servo.class, "gate");
        intakeRotateLeft = hwMap.get(Servo.class, "rotLeft");
        intakeRotateRight = hwMap.get(Servo.class, "rotRight");
        scoringBucket = hwMap.get(Servo.class, "bucket");
        blinkin = hwMap.get(RevBlinkinLedDriver.class, "blinkin");

        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoringLift.setDirection(DcMotor.Direction.FORWARD);
        scoringLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (resetEncoders) {
            hangLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scoringLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        hangLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLift.setDirection(DcMotor.Direction.REVERSE);
        hangLift.setDirection(DcMotor.Direction.FORWARD);
        scoringLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopMotors();

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
    }
    public void initNavigationIMU() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        navigationImu = hwMap.get(BNO055IMU.class, "imu 1");
        navigationImu.initialize(parameters);
        while (!navigationImu.isGyroCalibrated()){
            hwTelemetry.addData("Status", "Initializing IMU");
            opmode.sleep(50);
            opmode.idle();
        }
        navigationImu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        resetNavigationIMU();
        hwTelemetry.addData("Status", "IMU Initialized");
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontRightDrive.setZeroPowerBehavior(mode);
        frontLeftDrive.setZeroPowerBehavior(mode);
        backRightDrive.setZeroPowerBehavior(mode);
        backLeftDrive.setZeroPowerBehavior(mode);
    }

    public void wait(double seconds) {
        runtime.reset();
        while (this.opmode.opModeIsActive() && (runtime.seconds() < seconds)) {

        }
    }
    
    public double relativeNavigationHeading = 0.0;

    public void resetNavigationIMU() {
        readNavigationData();
        relativeNavigationHeading = navigationAngles.firstAngle;
        return;
    }

    public double getRelativeNavigationHeading() {
        double relativeHeading = 0;
        readNavigationData();
        relativeHeading = navigationAngles.firstAngle - relativeNavigationHeading;
        if (relativeHeading < -180.0) {
            relativeHeading = relativeHeading + 360;
        } else if (relativeHeading > 180.0) {
            relativeHeading = relativeHeading - 360;
        }

        Log.d("ROR: Nav Head: ", Double.toString(navigationAngles.firstAngle));
        Log.d("ROR: Relative Head: ", Double.toString(relativeHeading));
        return (relativeHeading);

    }
    
    public void readNavigationData() {
        navigationAngles = navigationImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //navigationGravity = navigationImu.getGravity();
        return;
    }

    public double getGlobalNavigationHeading() {
        readNavigationData();
        return (navigationAngles.firstAngle);
    }


    public void turnAtPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    public void turnAtPowerForSeconds(double power, double seconds) {
        // start turning
        turnAtPower(power);
        //Wait for time to elapse.
        wait(seconds);
        // stop motors.
        stopMotors();
        return;

    }

    public void driveAtPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void curveAtPowerForSeconds(double leftPower, double rightPower, double seconds) {
        // Start motors
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        // wait for time to elapse.
        wait(seconds);
        stopMotors();
        return;
    }

    public void driveAtPowerForSeconds(double power, double seconds) {
        // Start motors
        driveAtPower(power);
        // wait for time to elapse.
        wait(seconds);
        stopMotors();
        return;
    }

    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        return;
    }

    public void startStrafeLeft(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    public void startStrafeRight(double power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(+power);
    }


    public void driveCounts(double power, int counts) {
        boolean done = false;

        //backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int leftStartPosition = backLeftDrive.getCurrentPosition();
        int rightStartPosition = backRightDrive.getCurrentPosition();

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);


        while (this.opmode.opModeIsActive() && !done) {
            if ((Math.abs(backLeftDrive.getCurrentPosition() - leftStartPosition) > counts) || (Math.abs(backRightDrive.getCurrentPosition() - rightStartPosition) > counts)) {
                done = true;
            }
            //hwTelemetry.addData("DriveCounts Right: ", backRightDrive.getCurrentPosition() );
            //hwTelemetry.addData("DriveCounts Left: ", backLeftDrive.getCurrentPosition() );
            //hwTelemetry.update();
        }

        stopMotors();
        return;
    }

    public void driveCountsMantainHeading(double power, int counts, double headingTarget) {
        boolean done = false;
        double currentHeading;
        double delta;
        double accuracy = 1.0;
        double leftDriveMultiplier = 1.0;
        double rightDriveMultiplier = 1.0;

        int leftStartPosition = backLeftDrive.getCurrentPosition();
        int rightStartPosition = backRightDrive.getCurrentPosition();

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);


        while (this.opmode.opModeIsActive() && !done) {
            if ((Math.abs(backLeftDrive.getCurrentPosition() - leftStartPosition) > counts) || (Math.abs(backRightDrive.getCurrentPosition() - rightStartPosition) > counts)) {
                stopMotors();
                done = true;
            }
            currentHeading = getGlobalNavigationHeading();
            delta = navigationDelta(currentHeading, headingTarget);

            // If we need to adjust the heading, slow down one side of the robot.
            if (Math.abs(delta) > accuracy) {

                leftDriveMultiplier = 1.0;
                rightDriveMultiplier = 1.0;

                Log.d("ROR:DriveHead ", "Power: " + Double.toString(power) + " Delta: " + Double.toString(delta));
                if (power > 0.0) {
                    if (delta > 0.0) {
                        leftDriveMultiplier = 0.5;
                    } else
                        rightDriveMultiplier = 0.5;

                }
                // Power is negative
                else {
                    if (delta > 0.0) {
                        rightDriveMultiplier = 0.5;
                    } else
                        leftDriveMultiplier = 0.5;

                }

                Log.d("ROR:DriveHead ", "Left Multiplier: " + Double.toString(leftDriveMultiplier) + " Right Multiplier: " + Double.toString(rightDriveMultiplier));
                frontLeftDrive.setPower(power * leftDriveMultiplier);
                frontRightDrive.setPower(power * rightDriveMultiplier);
                backLeftDrive.setPower(power * leftDriveMultiplier);
                backRightDrive.setPower(power * rightDriveMultiplier);


            }


            //hwTelemetry.addData("DriveCounts Right: ", backRightDrive.getCurrentPosition() );
            //hwTelemetry.addData("DriveCounts Left: ", backLeftDrive.getCurrentPosition() );
            //hwTelemetry.update();
        }

        stopMotors();
        return;
    }

    public void curveAtPowerForCounts(double leftPower, double rightPower, int counts) {
        boolean done = false;

        //backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int leftStartPosition = backLeftDrive.getCurrentPosition();
        int rightStartPosition = backRightDrive.getCurrentPosition();

        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);


        while (this.opmode.opModeIsActive() && !done) {
            if ((Math.abs(backLeftDrive.getCurrentPosition() - leftStartPosition) > counts) || (Math.abs(backRightDrive.getCurrentPosition() - rightStartPosition) > counts)) {
                done = true;
            }
            //hwTelemetry.addData("DriveCounts Right: ", backRightDrive.getCurrentPosition() );
            //hwTelemetry.addData("DriveCounts Left: ", backLeftDrive.getCurrentPosition() );
            //hwTelemetry.update();
        }

        stopMotors();
        return;
    }


    //-----------------------------------------------------------------------------------------
    // Drive functions using Navigation IMU
    //-----------------------------------------------------------------------------------------


    public void turnToHeadingGlobalWithAdjust(double power, double headingTarget, double timeOutSeconds, double accuracy) {

        double headingAdjustment = 10.0;
        turnToHeadingGlobal(power, headingTarget, timeOutSeconds, accuracy, headingAdjustment);
    }

    // Standard turn, no heading adjustment, not high accuracy.
    public void turnToHeadingGlobal(double power, double headingTarget, double timeOutSeconds) {
        // Force accuracy at 3.0
        double accuracy = 3.0;
        turnToHeadingGlobal(power, headingTarget, timeOutSeconds, accuracy, 0.0);
    }

    // headingAdjust is added to heading.
    public void turnToHeadingGlobal(double power, double headingTarget, double timeOutSeconds, double accuracy, double headingAdjust) {

        double currentHeading;

        double delta = 1000.0;
        double oldNavDelta = 0;
        boolean deltaIncreasing = false;
        boolean directionPositive = false;
        double currentPower = 0.0;
        boolean powerHigh = false;

        Log.d("ROR:TTHG: ", "Power: " + Double.toString(power) + " Target: " + Double.toString(headingTarget) + "Timout: " + Double.toString(timeOutSeconds) + " Accuracy: " + Double.toString(accuracy) + " Adjust: " + Double.toString(headingAdjust));

        headingTarget += headingAdjust;

        headingTarget = convertAngleToHeading(headingTarget);

        // Select turn direction and set power accordingly.
        currentHeading = getGlobalNavigationHeading();

        delta = navigationDelta(currentHeading, headingTarget);
        Log.d("ROR:cur Global Head:", Double.toString(currentHeading));
        Log.d("ROR:cur Target:", Double.toString(headingTarget));
        Log.d("ROR:cur Delta:", Double.toString(delta));


        // negative delta: power positive.
        power = Math.abs(power);
        if (delta > 0) {
            power *= -1;
        }

        if (power < 0) {
            directionPositive = true;
        }

        Log.d("ROR:cur Power:", Double.toString(power));
        Log.d("ROR:cur Nav Pos:", Boolean.toString(directionPositive));
        boolean done = false;

        if (testMode) {
            waitForKey("Before turn");
        }

        // Start at slower power.
        powerHigh = false;
        currentPower = (.8 * power);
        turnAtPower(currentPower);

        double minDelta = 1000;

        runtime.reset();
        while (this.opmode.opModeIsActive() && !done) {
            currentHeading = getGlobalNavigationHeading();
            delta = navigationDelta(currentHeading, headingTarget);
            // If getting close slow down, but don't do this over and over.
            if (Math.abs(delta) < 20.0) {
                if (powerHigh) {
                    turnAtPower(currentPower = (.8 * power));
                    powerHigh = false;
                    Log.d("ROR:TTHG: ", "Set power to " + Double.toString(currentPower));

                }
            } else {
                if (!powerHigh) {
                    turnAtPower(power);
                    currentPower = power;
                    powerHigh = true;
                    Log.d("ROR:TTHG: ", "Set power to " + Double.toString(currentPower));
                }
            }

            //Log.d("ROR:cur Global Head:", Double.toString(currentHeading));
            //Log.d("ROR:cur Target:", Double.toString(headingTarget));
            //Log.d("ROR:cur Delta:", Double.toString(delta));


            if (Math.abs(delta) < Math.abs(minDelta)) {
                Log.d("ROR: min delta: ", Double.toString(minDelta));
                minDelta = delta;
            }


            // If we already passed the turn point prepare to stop
            if (Math.abs(delta) > Math.abs(minDelta)) {
                if (Math.abs(delta) < 10.0) {
                    stopMotors();
                    Log.d("ROR:DeltaIncreasing", Double.toString(delta) + " MinDelta: " + Double.toString(minDelta));
                    deltaIncreasing = true;
                    Log.d("ROR:TTHG: ", "Stop 1: delta increasing." + " Target: " + Double.toString(headingTarget) + " Current Head: " + Double.toString(currentHeading) + " Delta: " + Double.toString(delta) + " Min delta: " + Double.toString(minDelta));
                    if (testMode) {
                        waitForKey("Delta Increasing!");
                    }
                    done = true;
                    break;

                }
            }
            if (Math.abs(delta) < accuracy) {
                stopMotors();
                Log.d("ROR:TTHG: ", "Stop 2: accuracy reached." + " Target: " + Double.toString(headingTarget) + " Current Head: " + Double.toString(currentHeading) + " Delta: " + Double.toString(delta) + " Min delta: " + Double.toString(minDelta));
                Log.d("ROR:TTHG:", "Stopping deltaIncreasing:" + Boolean.toString(deltaIncreasing));
                done = true;
                break;
            }
            if (runtime.seconds() > timeOutSeconds) {
                stopMotors();
                Log.d("ROR: TTHG", "Out of time");
                done = true;
                break;
            }


        }
        stopMotors();
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public double adjustHeading() {
        Orientation currentHeading;
        double adjustedHeading = 0;
        currentHeading = navigationImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return (adjustedHeading);
    }

    public void hangForCounts(double power, int counts) {

        int startPosition = hangLift.getCurrentPosition();

        hangLift.setPower(power);

        while (this.opmode.opModeIsActive() && (Math.abs(hangLift.getCurrentPosition() - startPosition) < counts)) {

        }
        hangLift.setPower(0);
    }

    public void waitForKey(String message) {
        hwTelemetry.addData("Info: ", message);
        hwTelemetry.addData("Press the \"A\" button.", "");
        hwTelemetry.update();
        Log.d("ROR:waitForKey ", message);
        // wait for key to be pressed
        while (this.opmode.opModeIsActive() && !this.opmode.gamepad1.a) {
            hwTelemetry.addData("Info: ", message);
            hwTelemetry.addData("Waiting for \"A\" button.", "");
            hwTelemetry.update();
        }
        //wait for key to be unpressed.
        while (this.opmode.opModeIsActive() && this.opmode.gamepad1.a) {
            hwTelemetry.addData("Info: ", message);
            hwTelemetry.addData("Waiting for NOT\"A\" button.", "");
            hwTelemetry.update();

        }

        hwTelemetry.addData("A was pressed and unpressed.", "");
        hwTelemetry.update();

    }

    public void outtakeStart() {
        intakeRotateLeft.setPosition(1.0);
        intakeRotateRight.setPosition(-1.0);
    }

    public void intakeStart() {
        intakeRotateLeft.setPosition(-1.0);
        intakeRotateRight.setPosition(1.0);
    }

    public void intakeStop() {
        intakeRotateLeft.setPosition(0.0);
        intakeRotateRight.setPosition(0.0);
    }


    public void outTakeForTime(double seconds) {
        outtakeStart();
        wait(seconds);
        intakeStop();

    }


    // Overload: Add parameter for power.
    public void outTakeForTime(double seconds, double power) {
        if (power < 0.0) power *= -1;
        sweeper.setPower(power);
        wait(seconds);
        intakeStop();
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public double adjustNavigationAngle(double angle) {
        boolean done = false;
        while (!done) {
            if (angle < -180) {
                angle += 360;
            } else {
                done = true;
            }
        }
        done = false;
        while (!done) {
            if (angle > 180) {
                angle -= 360;
            } else {
                done = true;
            }
        }
        Log.d("ROR: adjusted angle:", Double.toString(angle));

        return (angle);
    }

    /*
    double navigationDelta(double angleOne, double angleTwo){
        double angleDiff;
        double adjustedDiff;

        angleDiff = angleTwo - angleOne;
        if (Math.abs(angleDiff)<180){
            Log.d("ROR: NavDelta: ", Double.toString(angleDiff));
            return(angleDiff);
        }
        else{
            adjustedDiff = 360 - Math.abs(angleDiff);
            if (angleTwo < angleOne){
                adjustedDiff *= -1.0;
            }
            Log.d("ROR: Nav Delta Adj:", Double.toString(adjustedDiff));
            return(adjustedDiff);
        }
    }
    */
    public void ejectMarker() {
        // Drop intake
        intakeDown();
        // Wait for servos to move
        wait(0.2f);
        // Outtake
        sweeperPower(-1);
        // Pick up intake
    }

    public double navigationDelta(double angleOne, double angleTwo) {
        double angleDiff;
        double adjustedDiff;


        // If either angle is negative convert to positive.
        angleOne = convertAngleToHeading(angleOne);
        angleTwo = convertAngleToHeading(angleTwo);


        angleDiff = angleTwo - angleOne;
        if (Math.abs(angleDiff) <= 180) {
            Log.d("ROR:NavDelta: ", "angleTwo: " + Double.toString(angleTwo) + " angleOne: " + Double.toString(angleOne) + " Diff:" + Double.toString(angleDiff));
            return (angleDiff);
        } else {
            boolean positive = true;
            if (angleDiff >= 0) positive = true;
            else positive = false;
            adjustedDiff = 360 - Math.abs(angleDiff);
            if (positive) {
                adjustedDiff *= -1.0;
            }
            Log.d("ROR:NavDelta: Adj:", "angleTwo: " + Double.toString(angleTwo) + " angleOne: " + Double.toString(angleOne) + " Diff:" + Double.toString(adjustedDiff));
            return (adjustedDiff);
        }
    }

    public double convertAngleToHeading(double angle) {
        Log.d("ROR:Convert angle: ", Double.toString(angle));

        angle %= 360;

        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;

        Log.d("ROR:Converted: ", Double.toString(angle));


        return angle;
    }

    public double batteryVoltage;
    public double turnPower = .45;
    public double drivePower = .40;

    // On plywood on carpet
    public double turnCoefficient = 1.10;
    public double driveCoefficient = 1.10;

    public void adjustPowerByVoltage() {
        Log.d("ROR: Bat Voltage ", Double.toString(batteryVoltage = getBatteryVoltage()));

        if (batteryVoltage > 14.0) {
            turnPower = .45 * turnCoefficient;
            drivePower = .4 * driveCoefficient;
        } else if (batteryVoltage > 13.5) {
            turnPower = .5 * turnCoefficient;
            drivePower = .425 * driveCoefficient;
            ;
        } else if (batteryVoltage > 13.0) {
            turnPower = .55 * turnCoefficient;
            drivePower = .45 * driveCoefficient;
        } else if (batteryVoltage > 12.0) {
            turnPower = .6 * turnCoefficient;
            drivePower = .5 * driveCoefficient;
        } else {
            turnPower = .7 * turnCoefficient;
            drivePower = .53 * driveCoefficient;
        }

        Log.d("ROR: TurnPower:", Double.toString(turnPower));
        Log.d("ROR: DrivePower:", Double.toString(drivePower));

    }
/*
    public void gateOpen(){
        gate.setPosition(1.0);
    }

    public void gateClose(){
        gate.setPosition(-1.0);
    }

*/

    public void setHangLift(float power) {
        hangLift.setPower(power);
    }

    public void runScoringLift(float power) {
        scoringLift.setPower(power);
    }

    public void gateOpen() {
        gate.setPosition(0.32);
    }

    public void gateClose() {
        gate.setPosition(0.7);
    }

    public void intakeDown() {
        intakePosition(0.77f);
    }

    public void intakeUp() {
        intakePosition(0.30f);
    }

    public void bucketScore() {
        scoringBucket.setPosition(0.67f);
    }

    public void bucketCollect() {
        scoringBucket.setPosition(0.2f);
    }

    public int encoderX() {
        return (frontLeftDrive.getCurrentPosition() / 16);
    }

    public int encoderY() {
        return (frontRightDrive.getCurrentPosition() / 16);
    }

    public void sendHangLiftToPos(int pos, float power, float timeOutS) {
        boolean done = false;

        if (pos > 0) {
            int end = getHangLiftPos() + pos;
            runtime.reset();
            while (done == false && opmode.opModeIsActive() && runtime.seconds() < timeOutS) {
                int currentPos = getHangLiftPos();
                if (currentPos < end) {
                    hangLift.setPower(power);
                }
                if (currentPos >= end) {
                    hangLift.setPower(0.0);
                    done = true;
                }
            }
        } else if (pos < 0) {
            int end = getHangLiftPos() + pos;
            while (done == false && opmode.opModeIsActive() && runtime.seconds() < timeOutS) {
                int currentPos = getHangLiftPos();
                if (currentPos > end) {
                    hangLift.setPower(power);
                }
                if (currentPos <= end) {
                    hangLift.setPower(0.0);
                    done = true;
                }
            }
        }
    }

    public void land() {
        boolean done = false;
        int pos = 3000;
        float power = -1;
        float timeOutS = 5f;

        if (pos > 0) {
            int end = pos;
            runtime.reset();
            while (done == false && opmode.opModeIsActive() && runtime.seconds() < timeOutS) {
                int currentPos = getHangLiftPos();
                if (currentPos < end) {
                    hangLift.setPower(power);
                }
                if (currentPos >= end) {
                    hangLift.setPower(0.0);
                    done = true;
                }
            }
        } else if (pos < 0) {
            int end = pos;
            while (done == false && opmode.opModeIsActive() && runtime.seconds() < timeOutS) {
                int currentPos = getHangLiftPos();
                if (currentPos > end) {
                    hangLift.setPower(power);
                }
                if (currentPos <= end) {
                    hangLift.setPower(0.0);
                    done = true;
                }
            }
        }
    }


    public void sweeperPower(float power) {
        sweeper.setPower(power);
    }

    public void setIntakeLiftPower(float power) {
        intakeLift.setPower(power);
    }

    public int getHangLiftPos() {
        return hangLift.getCurrentPosition();
    }

    public void intakePosition(float position) {
        intakeRotateLeft.setPosition(1 - position);
        intakeRotateRight.setPosition(position);
    }

    public void resetTeleWheelEncoders() {
        resetEncoderX();
        resetEncoderY();
    }

    public void resetEncoderX() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoderY() {
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void leftMotorPower(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    public void rightMotorPower(double power) {
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void encoderStrafeNew(float speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        // Ensure that the opmode is still active

        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightTarget = frontRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);


            // reset the timeout time and start motion.
            runtime.reset();


            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.abs(frontLeftDrive.getCurrentPosition()) < newLeftTarget) &&
                    (Math.abs(frontRightDrive.getCurrentPosition()) < newRightTarget) &&
                    (Math.abs(backLeftDrive.getCurrentPosition()) < newBackLeftTarget) &&
                    (Math.abs(backRightDrive.getCurrentPosition()) < newBackRightTarget)) {


                frontLeftDrive.setPower(speed);
                frontRightDrive.setPower(-speed);
                backLeftDrive.setPower(-speed);
                backRightDrive.setPower(speed);


                hwTelemetry.addData("Status", "Running to position");

                hwTelemetry.addData("FL", frontLeftDrive.getCurrentPosition() + " : " + newLeftTarget);
                hwTelemetry.addData("FR", frontRightDrive.getCurrentPosition() + " : " + newRightTarget);
                hwTelemetry.addData("BL", backLeftDrive.getCurrentPosition() + " : " + newBackLeftTarget);
                hwTelemetry.addData("BR", backRightDrive.getCurrentPosition() + " : " + newBackRightTarget);


                hwTelemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);


        }
    }

    public int getIntakeLiftPos() {
        return intakeLift.getCurrentPosition();
    }

    public void driveMotorPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void moveIntakeToPos(int pos, float power, float timeOutS) {
        intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean done = false;

        if (pos > 0) {
            int end = getIntakeLiftPos() + pos;
            runtime.reset();
            while (done == false && opmode.opModeIsActive() && runtime.seconds() < timeOutS) {
                int currentPos = getIntakeLiftPos();
                Log.d("12357", Integer.toString(currentPos));
                if (currentPos < end) {
                    intakeLift.setPower(power);
                }
                if (currentPos >= end) {
                    intakeLift.setPower(0.0);
                    done = true;
                }
            }
        } else if (pos < 0) {
            int end = getIntakeLiftPos() + pos;
            while (done == false && opmode.opModeIsActive() && runtime.seconds() < timeOutS) {
                int currentPos = getIntakeLiftPos();
                Log.d("12357", Integer.toString(currentPos));
                if (currentPos > end) {
                    intakeLift.setPower(power);
                }
                if (currentPos <= end) {
                    intakeLift.setPower(0.0);
                    done = true;
                }
            }
        }
    }

    public void waitMills(long millis) {
        try {
            sleep(millis);
        } catch (InterruptedException e) {
            hwTelemetry.addData("Error", "Failed to wait");
            hwTelemetry.update();
        }
    }


}

