package org.firstinspires.ftc.teamcode.Disabled;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.HardwareROR;

@TeleOp(name = "P Tuner Dashboard")
@Disabled
public class PTuner extends LinearOpMode {

    HardwareROR robot = new HardwareROR();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime sleepTimer = new ElapsedTime();
    Variables var = new Variables();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    int s4tCPR = 250;
    double teleWheelDiameterMM = 60, mmPerInch = 25.4f;
    double teleWheelDiameterIN = teleWheelDiameterMM / mmPerInch;
    double teleWheelCircumf = teleWheelDiameterIN * 3.141592;
    double ticksPerIN = s4tCPR / teleWheelCircumf;

    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    int teleError = 0;

    double moveError = var.moveError * ticksPerIN;

    double pX = var.pX, pY = var.pY, pYcorr = var.pYcorr, pCorr = var.pCorr;
    double minDrivePower = var.minDrivePower, minStrafePower = var.minStrafePower;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this, telemetry, false, true);
        robot.initNavigationIMU();
        robot.intakePosition(0.1f);
        while (!isStarted()) {
            telemetry.addData("TicksPerInch", ticksPerIN);
            telemetry.addData("Error", moveError);
            telemetry.update();
        }
        waitForStart();
        resetAngle();
        robot.resetTeleWheelEncoders();
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        strafe((float) var.driveDistance, (float) var.power, 10f);

        while (opModeIsActive()){
            robot.intakeDown();
        } ;

//        while (opModeIsActive()) {
//            telemetry.addData("pX", pX);
//            telemetry.addData("pY", pY);
//            telemetry.addData("pYcorr", pYcorr);
//            telemetry.addData("pCorr", pCorr);
//            telemetry.addData("minDrivePower", minDrivePower);
//            telemetry.addData("minStrafePower", minStrafePower);
//            telemetry.addData("power", var.drivePower);
//            telemetry.update();
//
//            drive(var.driveDistance, 0.5f, 5f);
//            waitForSecs(1f);
//            drive(-var.driveDistance, 0.5f, 5f);
//            waitForSecs(1f);
//
////            strafe(var.driveDistance, var.power, 5f);
////            waitForSecs(var.waitTime);
////            strafe(-var.driveDistance, var.power, 5f);
////            waitForSecs(var.waitTime);
//
//
//        }

    }

    private void waitForSecs(double seconds) {
        sleepTimer.reset();
        while (sleepTimer.seconds() < seconds && opModeIsActive()) {
            telemetry.addData("Status", "Waiting");
            telemetry.update();
            this.sleep(250);
        }
        telemetry.addData("Status", "Resuming");
        telemetry.update();
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

    private double encoderDrivePower(int targetTicks, double maxPower) {
        int direction = 1;
        if (targetTicks < 0){
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
        if (targetTicks < 0){
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
        while (opModeIsActive() && !isStopRequested() && runtime.seconds() < timeOutS) {
            int encoderX = robot.encoderX();
            if ((encoderX > targetTicks + moveError) || (encoderX < targetTicks - moveError)) {
                motorPower = encoderDrivePower((int) targetTicks, power);
                telemetry.addData("Status", "Driving using tele-wheels");
                telemetry.addData("Current Ticks", encoderX);
                telemetry.addData("Target Ticks", targetTicks);
                telemetry.addData("Current Motor Power", motorPower);

                correction = checkDirection();
                robot.leftMotorPower(motorPower + correction);
                robot.rightMotorPower(motorPower - correction);
                telemetry.update();

                Log.d("12357_Drive", "Global Heading " + Double.toString(globalAngle));
                Log.d("12357_Drive", "Correction " + Double.toString(correction));


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
        while (opModeIsActive() && !isStopRequested() && ((robot.encoderY() > targetTicks + moveError) || (robot.encoderY() < targetTicks - moveError)) && runtime.seconds() < timeOutS) {
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

            Log.d("12357:Strafe", "Front Right" + Double.toString(-motorPower - correction + encoderStrafeCorrection()));
            Log.d("12357:Strafe", "Back Left" + Double.toString(-motorPower + correction + encoderStrafeCorrection()));
            Log.d("12357:Strafe", "Front Left" + Double.toString(motorPower + correction + encoderStrafeCorrection()));
            Log.d("12357:Strafe", "Back Right" + Double.toString(motorPower - correction + encoderStrafeCorrection()));
        }
        robot.driveMotorPower(0);
    }
}
