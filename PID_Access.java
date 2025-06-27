package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PID_Access {

    // Define Variables, Pinpoint and Motors
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    GoBildaPinpointDriver odo;
    public static double xPodOffset = 148.9;
    public static double yPodOffset = -152.5;

    public static double hKp = 0.05;
    public static double hKi = 0;
    public static double hKd = 0.0;

    public static double yKp = 0.205;
    public static double yKi = 0;
    public static double yKd = 0.005224037;

    public static double xKp = 0.121503;
    public static double xKi = 0;
    public static double xKd = 0.008;


    //Target Position and Initial Position are in reference to the left origin point



    // Make instances of the PIDF Controller Class
    PIDFController xPID = new PIDFController(xKp, xKi, xKd, 0);
    PIDFController yPID = new PIDFController(yKp, yKi, yKd,0);
    PIDFController hPID = new PIDFController(hKp, hKi, hKd, 0);


public void initialize(HardwareMap hmap) {
    // Define and Configure Motors
    leftFront = hmap.get(DcMotorEx.class, "cm2");
    leftBack = hmap.get(DcMotorEx.class, "cm3");
    rightBack = hmap.get(DcMotorEx.class, "cm1");
    rightFront = hmap.get(DcMotorEx.class, "cm0");

    leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    // Define and Configure Pinpoint
    odo = hmap.get(GoBildaPinpointDriver.class, "odo");
    odo.setOffsets(xPodOffset, yPodOffset, DistanceUnit.MM);
    odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    odo.resetPosAndIMU();
}

public void moveToPose(Pose2D recInitialPosition, Pose2D recTargetPosition) {
    odo.update();
    double initialX = recInitialPosition.getX(DistanceUnit.INCH);
    double initialY=recInitialPosition.getY(DistanceUnit.INCH);
    double initialH=recInitialPosition.getHeading(AngleUnit.DEGREES);
    double targetX=recTargetPosition.getX(DistanceUnit.INCH);
    double targetY=recTargetPosition.getY(DistanceUnit.INCH);
    double targetH=recTargetPosition.getHeading(AngleUnit.DEGREES);




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
