package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;
@Config
@TeleOp
public class Waypoint extends LinearOpMode {

    FtcDashboard dashboard;

    Telemetry dashboardTelemetry;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private GoBildaPinpointDriver odo;

    public static double hKp = 0.05;
    public static double hKi = 0;
    public static double hKd = 0.0001;

    public static double yKp = 0.305;
    public static double yKi = 0.01;
    public static double yKd = 0.007224038;

    public static double xKp = 0.221503;
    public static double xKi = 0.01;
    public static double xKd = 0.026;

    public double initialX = 0;

    public static double initialY = 0;
    public static double initialH = 0;

    public static double targetX = 0;

    public static double targetY = 0;
    public static double targetH = 0;

    public static double xPodOffset = 148.9;

    public static double yPodOffset = -152.5;
    public static boolean canReset = false;


    PIDFController xPID = new PIDFController(xKp, xKi, xKd, 0);
    PIDFController yPID = new PIDFController(yKp, yKi, yKd,0);
    PIDFController hPID = new PIDFController(hKp, hKi, hKd, 0);

    //Target Position and Initial Position are in reference to the left origin point
    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        dashboardTelemetry = dashboard.getTelemetry();

        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        // Define and Configure Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "cm2");
        leftBack = hardwareMap.get(DcMotorEx.class, "cm3");
        rightBack = hardwareMap.get(DcMotorEx.class, "cm1");
        rightFront = hardwareMap.get(DcMotorEx.class, "cm0");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Define and Configure Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(xPodOffset, yPodOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            final Pose2D targetPosition = new Pose2D(DistanceUnit.INCH, targetX,targetY, AngleUnit.DEGREES, targetH);
            final Pose2D initialPosition = new Pose2D(DistanceUnit.INCH, initialX,initialY , AngleUnit.DEGREES, initialH);
            double deltaX = targetPosition.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY = targetPosition.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH = targetPosition.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition = new Pose2D(DistanceUnit.INCH, deltaX,deltaY , AngleUnit.DEGREES, deltaH);
            Pose2D odoPosition = odo.getPosition();
            double xPower = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition.getX(DistanceUnit.INCH));
            double yPower = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition.getY(DistanceUnit.INCH));
            double hPower = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition.getHeading(AngleUnit.DEGREES));
            driveFieldCentric(xPower, -yPower, -hPower);
            String data = String.format(Locale.US, "{X: %.1f, Y: %.1f, H: %.1f}", odoPosition.getX(DistanceUnit.INCH), odoPosition.getY(DistanceUnit.INCH), odoPosition.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Delta X:", deltaX);
            telemetry.addData("Delta Y:", deltaY);
            telemetry.addData("Delta H:", deltaH);
            telemetry.update();

        }
    }


    public void driveFieldCentric(double xPower, double yPower, double turnPower) {
        double headingRadians = odo.getHeading(AngleUnit.RADIANS);
        double strafe = yPower * Math.cos(-headingRadians) - xPower * Math.sin(-headingRadians);
        double forward = yPower * Math.sin(-headingRadians) + xPower * Math.cos(-headingRadians);
        driveRobotCentric(forward, strafe, turnPower);
    }
    /**
     * - Drives using the robots coordinate system
     * - Regular Game Pad Code
     *
     * @param axial   - strafe
     * @param lateral - forward/backward
     * @param yaw     - heading
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        double max;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }





}
