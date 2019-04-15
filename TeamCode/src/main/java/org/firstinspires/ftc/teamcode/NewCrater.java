package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class NewCrater {

    HardwareROR robot = new HardwareROR();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    OpenCVSample cv = new OpenCVSample();
    ElapsedTime sleepTimer = new ElapsedTime();
    Variables var = new Variables();
    Variables2 var2 = new Variables2();
    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();


    private double globalAngle, correction;
    private int teleError = 0;
    private double pX = 0.00005, pY = 0.00015, pYcorr = 0.00008, pRot = 0.02, pCorr = 0.035;
    private double minDrivePower = 0.19, minStrafePower = 0.4, minRotPower = 0.1;

    private int s4tCPR = 250;
    private double teleWheelDiameterMM = 60, mmPerInch = 25.4f;
    private double teleWheelDiameterIN = teleWheelDiameterMM / mmPerInch;
    private double teleWheelCircumf = teleWheelDiameterIN * 3.141592;
    private double ticksPerIN = s4tCPR / teleWheelCircumf;
    private String center = "CENTER", left = "LEFT", right = "RIGHT", none = "NONE";
    private String currentGoldPos = none;
    private String lastGoldPos = none;

    private double moveError = 0.5 * ticksPerIN;

    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private String teamNo;
    private Telemetry telemetry;

    private float leftRot = (float) var2.leftRot, rightRot = (float) var2.rightRot, scoringRot = (float) var2.scoringRot;

    int scoringLiftMult;

    private float driveToScore = 1.5f;

    private float driveToSampleDist = (float) var2.driveToSampleDist;

    protected void runAuto(HardwareMap hwMap, Telemetry t, LinearOpMode op, String team) {

        telemetry = t;
        opMode = op;
        hardwareMap = hwMap;
        teamNo = team;

        if (teamNo.equals("5667")) {

            scoringLiftMult = -1;
            driveToSampleDist = -42f;
            scoringRot = 12f;
            rightRot = 32;
            leftRot = -45;

        } else if (teamNo.equals("12357")) {

            scoringLiftMult = 1;
            rightRot = 32;
            leftRot = -49;
            driveToSampleDist = -39.5f;

        } else if (teamNo.equals("12384")) {

            scoringLiftMult = 1;
            scoringRot = 14f;
            rightRot = 39;
            leftRot = -48;
            driveToSampleDist = -43f;
            driveToScore = 0.5f;

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
            telemetry.addData("Status", "Ready to run " + teamNo + " crater");
            cv.sample();
            telemetry.update();
        }
        // Adjust Heading
        globalAngle += 45;
        // Land
        robot.land();
        // Strafe into lander briefly
        strafe(-1, 0.6f, 0.25f);
        // Drive off latch
        drive(2f, 0.5f, 5f);
        // Hold intake back
        robot.setIntakeLiftPower(-0.2f);
        // Rotate towards wall
        rotateForSecs(45, 1f, 0.5f);
        // Drive to wall
        drive(37, 1f, 2f);
        // Rotate towards depot
        rotateForSecs(-90, 1.5f, 0.5f);
        // Drive to depot
        drive(22, 0.6f, 3);
        // Extend lift
        moveIntakeOutToPosNoP(-350, 1, 2f);
        // Drop intake
        robot.ejectMarker();
        // Retract intake
        moveIntakeToPos(0, 0.8f, 1f);
        robot.setIntakeLiftPower(-0.2f);
        // Pick up intake
        robot.intakeUp();
        // Drive back from depot
        drive(-1.5f, 0.5f, 3);
        // Stop sweeper
        robot.sweeperPower(0);
        // Pick up intake
        robot.intakeUp();
        // Wait for servos to move
        waitForSecs(0.2);
        // Rotate towards sample field
        rotateForSecs(45, 1f, 0.5f);
        // Drive back to samples
        drive(driveToSampleDist, 0.7f, 3f);
        // Rotate to sample
        if (cv.lastGoldPos == left)
            rotateForSecs(90 + leftRot, 1f, 1f);
        if (cv.lastGoldPos == center || cv.lastGoldPos == none) {
            rotateForSecs(90, 1f, 1f);
            drive(-2, 0.5f, 1f);
        }
        if (cv.lastGoldPos == right)
            rotateForSecs(90 + rightRot, 1.3f, 1f);
        // Drop and start intake
        robot.intakeDown();
        robot.sweeperPower(1);
        // Wait for servos to move
        waitForSecs(0.2);
        // Extend intake and drop scoring bucket
        moveScoringAndIntakeToPos(-250, 0.5f, -170 * scoringLiftMult, 1, 1.25f);
        // Retract intake
        moveIntakeToPos(0, 1, 1f);
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
        // Rotate to scoring position
        if (cv.lastGoldPos == left)
            rotateForSecs(-leftRot + scoringRot, 1f, 1f);
        if (cv.lastGoldPos == center || cv.lastGoldPos == none)
            rotateForSecs(scoringRot, 1f, 1f);
        if (cv.lastGoldPos == right)
            rotateForSecs(-rightRot + scoringRot, 1f, 1f);
        // Move intake forward a little
        moveIntakeOutToPosNoP(-20, 0.2f, 0.5f);
        // Wait for servos to move
        waitForSecs(0.2);
        // Close gateXX
        robot.gateClose();
        // Drive forwards
        drive(driveToScore, 0.4f, 2f);
        // Extend scoring lift and push intake out a little
        moveScoringAndIntakeToPos(-10, 0.2f, 800 * scoringLiftMult, 1f, 1.5f);
        // Hold lift up
        robot.runScoringLift(-1f);
        // Tilt bucket
        robot.bucketScore();
        // Close gate
        robot.gateClose();
        // Pick up intake
        robot.intakeUp();
        // Wait for mineral to fall
        waitForSecs(0.7f);
        // Untilt bucket
        robot.bucketCollect();
        // Cut scoring lift power
        robot.runScoringLift(0);
        // Bring lift back down and put intake back into crater
        moveScoringAndIntakeToPos(-350, 1, 0, 1, 2f);
        // Drop intake
        robot.intakeDown();
        // Wait for servos to move
        waitForSecs(0.2);
        // Pull intake back
        moveIntakeToPos(-250, 1f, 0.5f);
        // Extend intake into crater
        moveIntakeToPos(-500, 0.3f, 1.5f);
        // Pull intake back
        moveIntakeToPos(-250, 1f, 0.5f);
        // Extend intake into crater
        moveIntakeToPos(-500, 0.3f, 1.5f);
        // Pull back intake
        moveIntakeToPos(-250, 1f, 0.5f);
        // Pick up intake
        robot.intakeUp();
        // Pull intake all the way back
        moveIntakeToPos(0, 1, 1f);
        // Open gate
        robot.gateOpen();
        // Pull intake in
        robot.setIntakeLiftPower(-0.2f);
        // Wait for minerals to fall
        waitForSecs(1.2);
        // Strafe a little
        strafe(1f, 0.8f, 1f);
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
        // Pick up intake
        robot.intakeUp();
        // Wait for mineral to fall
        waitForSecs(0.7f);
        // Untilt bucket
        robot.bucketCollect();
        // Cut scoring lift power
        robot.runScoringLift(0);
        // Bring scoring lift down
        moveScoringAndIntakeToPos(-10, 0.2f, 0, 1f, 2f);
        // Rotate a little to new part of crater
        rotateForSecs(-25, 0.5f, 0.7f);
        // Put intake into crater
        moveIntakeToPos(-250, 1, 2f);
        // Drop intake
        robot.intakeDown();
        // Extend intake into crater
        moveIntakeToPos(-500, 0.7f, 1f);
        // Pull back intake
        moveIntakeToPos(-250, 0.3f, 1f);
        // Extend intake into crater
        moveIntakeToPos(-500, 0.7f, 1f);


        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            waitForSecs(0.005);
        }
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
