package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class NewDepot {

    HardwareROR robot = new HardwareROR();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();
    Orientation lastAngles = new Orientation();
    OpenCVSample cv = new OpenCVSample();
    ElapsedTime sleepTimer = new ElapsedTime();
    Variables var = new Variables();

    double globalAngle, correction;
    int teleError = 0;
    double pX = 0.00008, pY = 0.00004, pYcorr = 0.00008, pRot = 0.015, pCorr = 0.03;
    double minDrivePower = 0.2, minStrafePower = 0.4, minRotPower = 0.2;

    int s4tCPR = 250;
    double teleWheelDiameterMM = 60, mmPerInch = 25.4f;
    double teleWheelDiameterIN = teleWheelDiameterMM / mmPerInch;
    double teleWheelCircumf = teleWheelDiameterIN * 3.141592;
    double ticksPerIN = s4tCPR / teleWheelCircumf;
    private String center = "CENTER", left = "LEFT", right = "RIGHT", none = "NONE";

    private double moveError = 0.5 * ticksPerIN;

    private float leftRot = 42, rightRot = 42;

    private LinearOpMode opMode;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private String teamNo;

    int scoringLiftMult;

    float firstDriveBack = -3f;
    float driveBackFromDepot = -11f;
    float scoreCorrection = 0f;
    float scoreDrive = 8f;
    float lastStrafe = 5f;

    protected void runAuto(HardwareMap hwMap, Telemetry t, LinearOpMode op, String team) {

        telemetry = t;
        opMode = op;
        hardwareMap = hwMap;
        teamNo = team;

        if (teamNo.equals("5667")) {

            scoringLiftMult = -1;
            rightRot = 42;
            leftRot = 42;

        } else if (teamNo.equals("12357")) {

            scoringLiftMult = 1;
            rightRot = 32;
            leftRot = 36;
            firstDriveBack = 0f;
            driveBackFromDepot = -8f;
            scoreCorrection = 2.5f;
            scoreDrive = 9f;
            lastStrafe = 3f;

        } else if (teamNo.equals("12384")) {

            scoringLiftMult = 1;
            rightRot = 38;
            leftRot = 38;
            firstDriveBack = -3f;
            driveBackFromDepot = -11f;
            scoreCorrection = 0;
            scoreDrive = 11f;
            lastStrafe = 5.5f;

        }

        robot.init(hardwareMap, opMode, telemetry, false, true);
        robot.initNavigationIMU();
        resetAngle();
        robot.intakePosition(0.1f);
        cv.init(hardwareMap, opMode, telemetry);
        robot.bucketCollect();
        robot.gateClose();
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!opMode.opModeIsActive() && !opMode.isStopRequested()) {
            telemetry.addData("Status", "Ready to run " + teamNo + " depot");
            cv.sample();
            telemetry.update();
        }
        // Reset total timer
        totalTime.reset();
        // Adjust Heading
        globalAngle += 45;
        // Hold intake lift back
        robot.setIntakeLiftPower(-0.2f);
        // Drop from lander
        robot.land();
        // Drive off latch
        drive(3f, 0.5f, 5f);
        // Strafe away from lander
        strafe(2, 0.8f, 3f);
        // Drive back to center
        drive(firstDriveBack, 0.5f, 5f);
        // Rotate towards depot
        rotateForSecs(90, 1.5f, 1f);
        // Drive up to samples
        drive(16, 0.5f, 3f);
        // Entend Lift
        moveIntakeOutToPosNoP(-400, 1, 1);
        // Eject Marker
        robot.ejectMarker();
        // Wait for servos to move
        waitForSecs(0.3f);
        // Pick up intake
        robot.intakeUp();
        // Wait for servos to move
        waitForSecs(0.3f);
        // Bring intake back and drive back a little
        moveIntakeToPos(0, 1, 1.5f);
        // Hold intake lift back
        robot.setIntakeLiftPower(-0.2f);
        // Drive back
        drive(driveBackFromDepot, 0.7f, 2f);
        // Rotate towards sample
        if (cv.lastGoldPos == left) {
            rotateForSecs(-leftRot, 1f, 1);
        } else if (cv.lastGoldPos == center || cv.lastGoldPos == none) {
            // Hey!! We're already lined up! Would ya look at that!
        } else if (cv.lastGoldPos == right) {
            rotateForSecs(rightRot, 1f, 1);
        }
        // Make sure gate is closed
        robot.gateClose();
        // Drop and start intake
        robot.intakeDown();
        robot.sweeperPower(1);
        // Wait for servos to move
        waitForSecs(0.2);
        // Extend intake and drop scoring bucket
        moveScoringAndIntakeToPos(-250, 0.3f, -170 * scoringLiftMult, 1, 4f);
        // Retract intake
        moveIntakeToPos(0, 0.5f, 1f);
        if (cv.lastGoldPos == center || cv.lastGoldPos == none)
            drive(2.5f, 0.5f, 1f);
        // Reset scoring lift encoder now that it is at bottom
        robot.scoringLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.scoringLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Pick up intake
        robot.intakeUp();
        // Pull intake in
        robot.setIntakeLiftPower(-0.2f);
        // Open gate
        robot.gateOpen();
        // Rotate back to center
        if (cv.lastGoldPos == left) {
            rotateForSecs(leftRot, 1f, 1);
        } else if (cv.lastGoldPos == center || cv.lastGoldPos == none) {
            // Hey!! We're already lined up! Would ya look at that!
        } else if (cv.lastGoldPos == right) {
            rotateForSecs(-rightRot, 1f, 1);
        }
        // Stop intaking
        robot.sweeperPower(0);
        // Drive forward a little
        drive(scoreDrive, 0.5f, 3f);
        // Strafe a little to the left
        strafe(scoreCorrection, 0.7f, 2f);
        // Move intake out a little
        moveIntakeOutToPosNoP(-20, 0.2f, 0.5f);
        // Extend scoring lift
        moveScoringAndIntakeToPos(-10, 0.2f, 800 * scoringLiftMult, 1f, 1.5f);
        // Hold lift up
        robot.runScoringLift(-1f);
        // Tilt bucket
        robot.bucketScore();
        // Close gate
        robot.gateClose();
        // Wait for mineral to fall
        waitForSecs(0.7);
        // Untilt bucket
        robot.bucketCollect();
        // Cut scoring lift power
        robot.runScoringLift(0);
        // Bring lift back down
        moveScoringAndIntakeToPos(-10, 0.2f, 0, 1, 1.5f);
        // Backup a tiny bit
        minDrivePower = 0.3;
        drive(-4, 1, 2);
        minDrivePower = 0.2;
        // Rotate towards crater
        rotateForSecs(-90, 1f, 1);
        // Strafe away from lander leg
        strafe(lastStrafe, 0.8f, 2f);
        // Hold intake back
        robot.setIntakeLiftPower(-0.2f);
        // Drive to crater
        drive(44, 0.7f, 3);
        // Rotate towards crater
        rotateForSecs(-45, 1f, 1);
        // Strafe towards wall
        strafe(15, 1, 1f);
        // Drive into crater
        drive(5, 1, 1f);
        // Entend intake over crater
        moveIntakeToPos(-230, 0.5f, 1f);
        // Start sweeper
        robot.sweeperPower(1);
        // Drop intake
        robot.intakeDown();
        // Wait for servos to move
        opMode.sleep(1000);
        // Bring intake back a little
        moveIntakeToPos(-150, 0.5f, 1.5f);
        // Entend intake into crater
        moveIntakeToPos(-550, 0.5f, 1.5f);

        while (opMode.opModeIsActive() && !opMode.isStopRequested() && totalTime.seconds() < 26.5) {
            telemetry.addData("Time Remaining", totalTime.seconds());
            moveIntakeToPos(-150, 0.5f, 1.2f);
            // Entend intake into crater
            moveIntakeToPos(-550, 0.5f, 1.2f);
        }
        robot.intakeUp();
        moveIntakeToPos(0, 0.8f, 1f);
        robot.setIntakeLiftPower(-0.2f);
        robot.gateOpen();
        waitForSecs(1.5f);
        robot.intakeDown();
        waitForSecs(0.1f);
        cv.detector.disable();
    }

    private void resetAngle() {
        lastAngles = robot.navigationImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = robot.navigationImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        double correction, angle, gain = pCorr;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = angle;

        correction = correction * gain;

        return correction;
    }

    private double rotatePower() {
        double power, angle, gain = pRot, minPower = minRotPower;

        angle = getAngle();

        if (angle == 0)
            power = 0;             // no adjustment.
        else
            power = angle;

        power = power * gain;

        if (Math.abs(power) < minPower) {
            if (power < 0)
                power = -minPower;
            if (power > 0)
                power = minPower;
        }

        return power;
    }

    private double encoderDrivePower(int targetTicks, double maxPower) {
        int direction = 1;
        if (targetTicks < 0) {
            direction *= -1;
        }
        targetTicks = Math.abs(targetTicks);
        double ticks, gain = pX, error = teleError, minPower = minDrivePower;
        double scale = (maxPower - minPower) / (0.7 * targetTicks);
        double ticksToTrav;
        double ticksForMin = (0.7 * targetTicks);

        ticks = Math.abs(robot.encoderX());
        ticksToTrav = (targetTicks - ticks) - (0.3 * targetTicks);

        if (ticks < targetTicks + error && ticks > targetTicks - error)
            var.drivePower = 0;             // no adjustment.
        else {
            var.drivePower = (scale * ticksToTrav) + minPower;
        }
        Log.d("ROR:Power", "Scale " + Double.toString(scale));
        Log.d("ROR:Power", "To travel " + Double.toString(ticksToTrav));
        Log.d("ROR:Power", "Ticks for min " + Double.toString(ticksForMin));
        Log.d("ROR:Power", "Power" + Double.toString(var.drivePower));

        if (ticks > ticksForMin) {
            var.drivePower = minDrivePower;
            Log.d("ROR:Power", "Min Power Override");
        }

        return var.drivePower * direction;
    }

    private double encoderStrafePower(int targetTicks, double maxPower) {
        int direction = 1;
        if (targetTicks < 0) {
            direction *= -1;
        }
        targetTicks = Math.abs(targetTicks);
        double ticks, gain = pX, error = teleError, minPower = minStrafePower;
        double scale = (maxPower - minPower) / (0.7 * targetTicks);
        double ticksToTrav;
        double ticksForMin = (0.7 * targetTicks);

        ticks = Math.abs(robot.encoderY());
        ticksToTrav = (targetTicks - ticks) - (0.3 * targetTicks);

        if (ticks < targetTicks + error && ticks > targetTicks - error)
            var.strafePower = 0;             // no adjustment.
        else {
            var.strafePower = (scale * ticksToTrav) + minPower;
        }
        Log.d("ROR:Power", "Scale " + Double.toString(scale));
        Log.d("ROR:Power", "To travel " + Double.toString(ticksToTrav));
        Log.d("ROR:Power", "Ticks for min " + Double.toString(ticksForMin));
        Log.d("ROR:Power", "Power" + Double.toString(var.strafePower));

        if (ticks > ticksForMin) {
            var.strafePower = minStrafePower;
            Log.d("ROR:Power", "Min Power Override");
        }

        return -var.strafePower * direction;
    }

    private double encoderStrafeCorrection() {
        double power, ticks, gain = pYcorr, error = 30;

        ticks = robot.encoderX();

        if (ticks < error && ticks > error)
            power = 0;             // no adjustment.
        else
            power = 0 - robot.encoderX();

        power = power * gain;
        return power;
    }

    private void drive(float inches, float power, float timeOutS) {
        robot.resetTeleWheelEncoders();

        double targetTicks = inches * ticksPerIN;
        double motorPower;

        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && runtime.seconds() < timeOutS) {
            int encoderX = Math.abs(robot.encoderX());
            if (encoderX < Math.abs(targetTicks) - moveError) {
                motorPower = encoderDrivePower((int) targetTicks, power);
                telemetry.addData("Status", "Driving using tele-wheels");
                telemetry.addData("Current Ticks", encoderX);
                telemetry.addData("Target Ticks", targetTicks);
                telemetry.addData("Current Motor Power", motorPower);

                correction = checkDirection();
                robot.leftMotorPower(motorPower + correction);
                robot.rightMotorPower(motorPower - correction);
                telemetry.update();

                Log.d("ROR_Drive", "Global Heading " + Double.toString(globalAngle));
                Log.d("ROR_Drive", "Correction " + Double.toString(correction));


            } else {
                break;
            }
        }
        robot.driveMotorPower(0);


    }

    private void strafe(float inches, float power, float timeOutS) {
        robot.resetTeleWheelEncoders();

        double targetTicks = -inches * ticksPerIN;
        double motorPower;

        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && (Math.abs(robot.encoderY()) < Math.abs(targetTicks) - moveError) && runtime.seconds() < timeOutS) {
            motorPower = encoderStrafePower((int) targetTicks, power);
            telemetry.addData("Status", "Strafing using tele-wheels");
            telemetry.addData("Current Ticks", robot.encoderY());
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Motor Power", motorPower);
            telemetry.addData("X Dir Strafe", encoderStrafeCorrection());
            correction = checkDirection();
            robot.frontRightDrive.setPower(-motorPower - correction + encoderStrafeCorrection());
            robot.backLeftDrive.setPower(-motorPower + correction + encoderStrafeCorrection());
            robot.frontLeftDrive.setPower(motorPower + correction + encoderStrafeCorrection());
            robot.backRightDrive.setPower(motorPower - correction + encoderStrafeCorrection());
            telemetry.update();

            Log.d("ROR:Strafe", "Front Right" + Double.toString(-motorPower - correction + encoderStrafeCorrection()));
            Log.d("ROR:Strafe", "Back Left" + Double.toString(-motorPower + correction + encoderStrafeCorrection()));
            Log.d("ROR:Strafe", "Front Left" + Double.toString(motorPower + correction + encoderStrafeCorrection()));
            Log.d("ROR:Strafe", "Back Right" + Double.toString(motorPower - correction + encoderStrafeCorrection()));
        }
        robot.driveMotorPower(0);
    }

    private void rotateForSecs(float degrees, float time, float power) {
        globalAngle += degrees;
        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && runtime.seconds() < time) {
            correction = rotatePower();
            if (Math.abs(correction) > power) {
                if (correction > 0)
                    correction = power;
                if (correction < 0)
                    correction = -power;
            }
            robot.leftMotorPower(correction);
            robot.rightMotorPower(-correction);
        }
        robot.driveMotorPower(0);
    }

    private float intakeLiftTarget(int targetPosition) {
        float power, position, gain = .015f, minPower = 0.1f;
        int error = 10;

        position = robot.intakeLift.getCurrentPosition();

        if (position < targetPosition + error && position > targetPosition - error)
            power = 0.0f;             // no adjustment.
        else
            power = targetPosition - robot.intakeLift.getCurrentPosition();

        power = power * gain;

        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;
        if (power < minPower && power > 0)
            power = minPower;
        if (power > -minPower && power < 0)
            power = -minPower;

        return -power;
    }

    private void moveIntakeToPos(int pos, float power, float timeOutS) {

        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && ((robot.getIntakeLiftPos() > pos + 10) || (robot.getIntakeLiftPos() < pos - 10)) && runtime.seconds() < timeOutS) {
            float intakeLiftPower = intakeLiftTarget(pos);
            telemetry.addData("Intake lift power", intakeLiftPower);
            if (intakeLiftPower > power) {
                intakeLiftPower = power;
            }
            if (intakeLiftPower < -power) {
                intakeLiftPower = -power;
            }
            robot.setIntakeLiftPower(intakeLiftPower);

            correction = checkDirection();
            telemetry.addData("Rotation Power", correction);
            robot.leftMotorPower(correction);
            robot.rightMotorPower(-correction);
            telemetry.update();
        }
        robot.setIntakeLiftPower(0);
    }

    private void moveIntakeOutToPosNoP(int pos, float power, float timeOutS) {
        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && robot.getIntakeLiftPos() > pos && runtime.seconds() < timeOutS) {
            float intakeLiftPower = power;
            telemetry.addData("Intake lift power", intakeLiftPower);

            robot.setIntakeLiftPower(intakeLiftPower);

            correction = checkDirection();
            telemetry.addData("Rotation Power", correction);
            robot.leftMotorPower(correction);
            robot.rightMotorPower(-correction);
            telemetry.update();
        }
        robot.setIntakeLiftPower(0);
    }

    private float scoringLiftTarget(int targetPosition) {
        float power, position, gain = .015f, minPower = 0.4f;
        int error = 10;

        position = robot.scoringLift.getCurrentPosition() * scoringLiftMult;

        if (position < targetPosition + error && position > targetPosition - error)
            power = 0.0f;             // no adjustment.
        else
            power = (targetPosition * scoringLiftMult) - (robot.scoringLift.getCurrentPosition() * scoringLiftMult);

        power = -power * gain;

        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;
        if (power < minPower && power > 0)
            power = minPower;
        if (power > -minPower && power < 0)
            power = -minPower;

        return power;
    }

    private void moveScoringAndIntakeToPos(int intakePos, float intakePower, int scoringPos, float scoringPower, float timeOutS) {
        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && runtime.seconds() < timeOutS &&
                ((robot.getIntakeLiftPos() > intakePos + 10) || (robot.getIntakeLiftPos() < intakePos - 10) ||
                        (robot.scoringLift.getCurrentPosition() * scoringLiftMult > scoringPos + 10) ||
                        (robot.scoringLift.getCurrentPosition() * scoringLiftMult < scoringPos - 10))) {
            float intakeLiftPower = intakeLiftTarget(intakePos);
            telemetry.addData("Intake lift power", intakeLiftPower);
            if (intakeLiftPower > intakePower) {
                intakeLiftPower = intakePower;
            }
            if (intakeLiftPower < -intakePower) {
                intakeLiftPower = -intakePower;
            }
            robot.setIntakeLiftPower(intakeLiftPower);

            float scoringLiftPower = scoringLiftTarget(scoringPos);
            telemetry.addData("Scoring lift power", scoringLiftPower);

            if (scoringLiftPower > scoringPower) {
                scoringLiftPower = scoringPower;
            }
            if (scoringLiftPower < -scoringPower) {
                scoringLiftPower = -scoringPower;
            }
            robot.runScoringLift(scoringLiftPower);

            correction = checkDirection();
            telemetry.addData("Rotation Power", correction);
            robot.leftMotorPower(correction);
            robot.rightMotorPower(-correction);
            telemetry.update();
        }
        robot.setIntakeLiftPower(0);
        robot.runScoringLift(0);
    }

    private void waitForSecs(double seconds) {
        sleepTimer.reset();
        while (sleepTimer.seconds() < seconds && opMode.opModeIsActive()) {
            telemetry.addData("Status", "Waiting");
            telemetry.update();
            opMode.sleep(50);
        }
        telemetry.addData("Status", "Resuming");
        telemetry.update();
    }
}
