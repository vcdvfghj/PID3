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
public class threeSpecDraft extends LinearOpMode {

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

    public double initialX = 17.5;

    public static double initialY = 37.8;
    public static double initialH = 0;

    public static double xPodOffset = 148.9;

    public static double yPodOffset = -152.5;

    boolean firstStepRan = false;
    boolean secondStepRan = false;
    boolean thirdStepRan = false;
    boolean fourthStepRan = false;
    boolean fifthStepRan = false;
    boolean sixthStepRan = false;
    boolean seventhStepRan = false;
    public static boolean CP1Aresume = false;
    public static boolean CP1Bresume = false;
    public static boolean CP2Aresume = false;
    public static boolean CP2Bresume = false;
    public static boolean CP3Aresume = false;
    public static boolean CP3Bresume = false;
    public static boolean CP4resume = false;
    public static boolean CP5resume = false;





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
            final Pose2D initialPosition = new Pose2D(DistanceUnit.INCH, initialX,initialY , AngleUnit.DEGREES, initialH);
            final Pose2D CP1A = new Pose2D(DistanceUnit.INCH, 29,22.5, AngleUnit.DEGREES, -33.4);
            final Pose2D CP1B = new Pose2D(DistanceUnit.INCH, 9.6,33, AngleUnit.DEGREES, -133.4);
            final Pose2D CP2A = new Pose2D(DistanceUnit.INCH, 29.3,11.8, AngleUnit.DEGREES, -33.4);
            final Pose2D CP2B = new Pose2D(DistanceUnit.INCH, 13.4,25.2, AngleUnit.DEGREES, -150);
            //Moving to 3A causes the robot to hit the wall, we can establish a waypoint later
            final Pose2D CP3A = new Pose2D(DistanceUnit.INCH, 28.9,1.3, AngleUnit.DEGREES, -33.4);
            final Pose2D CP3B = new Pose2D(DistanceUnit.INCH, 13.7,18.8, AngleUnit.DEGREES, -170);
            final Pose2D CP4 = new Pose2D(DistanceUnit.INCH, 23.2,22.4, AngleUnit.DEGREES, 0);
            final Pose2D CP5 = new Pose2D(DistanceUnit.INCH, 49.8,60.5, AngleUnit.DEGREES, 0);

            Pose2D odoPosition = odo.getPosition();

            double deltaX1A = CP1A.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY1A = CP1A.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH1A = CP1A.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition1A = new Pose2D(DistanceUnit.INCH, deltaX1A,deltaY1A , AngleUnit.DEGREES, deltaH1A);
            double xPower1A = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition1A.getX(DistanceUnit.INCH));
            double yPower1A = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition1A.getY(DistanceUnit.INCH));
            double hPower1A = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition1A.getHeading(AngleUnit.DEGREES));

            double deltaX1B = CP1B.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY1B = CP1B.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH1B = CP1B.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition1B = new Pose2D(DistanceUnit.INCH, deltaX1B,deltaY1B , AngleUnit.DEGREES, deltaH1B);
            double xPower1B = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition1B.getX(DistanceUnit.INCH));
            double yPower1B = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition1B.getY(DistanceUnit.INCH));
            double hPower1B = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition1B.getHeading(AngleUnit.DEGREES));

            double deltaX2A = CP2A.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY2A = CP2A.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH2A = CP2A.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition2A = new Pose2D(DistanceUnit.INCH, deltaX2A,deltaY2A , AngleUnit.DEGREES, deltaH2A);
            double xPower2A = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition2A.getX(DistanceUnit.INCH));
            double yPower2A = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition2A.getY(DistanceUnit.INCH));
            double hPower2A = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition2A.getHeading(AngleUnit.DEGREES));

            double deltaX2B = CP2B.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY2B = CP2B.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH2B = CP2B.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition2B = new Pose2D(DistanceUnit.INCH, deltaX2B,deltaY2B , AngleUnit.DEGREES, deltaH2B);
            double xPower2B = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition2B.getX(DistanceUnit.INCH));
            double yPower2B = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition2B.getY(DistanceUnit.INCH));
            double hPower2B = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition2B.getHeading(AngleUnit.DEGREES));

            double deltaX3A = CP3A.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY3A = CP3A.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH3A = CP3A.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition3A = new Pose2D(DistanceUnit.INCH, deltaX3A,deltaY3A , AngleUnit.DEGREES, deltaH3A);
            double xPower3A = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition3A.getX(DistanceUnit.INCH));
            double yPower3A = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition3A.getY(DistanceUnit.INCH));
            double hPower3A = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition3A.getHeading(AngleUnit.DEGREES));

            double deltaX3B = CP3B.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY3B = CP3B.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH3B = CP3B.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition3B = new Pose2D(DistanceUnit.INCH, deltaX3B,deltaY3B , AngleUnit.DEGREES, deltaH3B);
            double xPower3B = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition3B.getX(DistanceUnit.INCH));
            double yPower3B = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition3B.getY(DistanceUnit.INCH));
            double hPower3B = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition3B.getHeading(AngleUnit.DEGREES));

            double deltaX4 = CP4.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY4 = CP4.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH4 = CP4.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition4 = new Pose2D(DistanceUnit.INCH, deltaX4,deltaY4 , AngleUnit.DEGREES, deltaH4);
            double xPower4 = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition4.getX(DistanceUnit.INCH));
            double yPower4 = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition4.getY(DistanceUnit.INCH));
            double hPower4 = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition4.getHeading(AngleUnit.DEGREES));

            double deltaX5 = CP5.getX(DistanceUnit.INCH)-initialPosition.getX(DistanceUnit.INCH);
            double deltaY5 = CP5.getY(DistanceUnit.INCH)-initialPosition.getY(DistanceUnit.INCH);
            double deltaH5 = CP5.getHeading(AngleUnit.DEGREES)-initialPosition.getHeading(AngleUnit.DEGREES);
            Pose2D PinpointTargetPosition5 = new Pose2D(DistanceUnit.INCH, deltaX5,deltaY5, AngleUnit.DEGREES, deltaH5);
            double xPower5 = xPID.calculate(odoPosition.getX(DistanceUnit.INCH), PinpointTargetPosition5.getX(DistanceUnit.INCH));
            double yPower5 = yPID.calculate(odoPosition.getY(DistanceUnit.INCH), PinpointTargetPosition5.getY(DistanceUnit.INCH));
            double hPower5 = hPID.calculate(odoPosition.getHeading(AngleUnit.DEGREES), PinpointTargetPosition5.getHeading(AngleUnit.DEGREES));

            double differenceX1A = Math.abs(PinpointTargetPosition1A.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY1A = Math.abs(PinpointTargetPosition1A.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH1A = Math.abs(PinpointTargetPosition1A.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            double differenceX1B = Math.abs(PinpointTargetPosition1B.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY1B = Math.abs(PinpointTargetPosition1B.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH1B = Math.abs(PinpointTargetPosition1B.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            double differenceX2A = Math.abs(PinpointTargetPosition2A.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY2A = Math.abs(PinpointTargetPosition2A.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH2A = Math.abs(PinpointTargetPosition2A.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            double differenceX2B = Math.abs(PinpointTargetPosition2B.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY2B = Math.abs(PinpointTargetPosition2B.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH2B = Math.abs(PinpointTargetPosition2B.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            double differenceX3A = Math.abs(PinpointTargetPosition3A.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY3A = Math.abs(PinpointTargetPosition3A.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH3A = Math.abs(PinpointTargetPosition3A.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            double differenceX3B = Math.abs(PinpointTargetPosition3B.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY3B= Math.abs(PinpointTargetPosition3B.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH3B= Math.abs(PinpointTargetPosition3B.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            double differenceX4 = Math.abs(PinpointTargetPosition4.getX(DistanceUnit.INCH)-odoPosition.getX(DistanceUnit.INCH));
            double differenceY4= Math.abs(PinpointTargetPosition4.getY(DistanceUnit.INCH)-odoPosition.getY(DistanceUnit.INCH));
            double differenceH4= Math.abs(PinpointTargetPosition4.getHeading(AngleUnit.DEGREES)-odoPosition.getHeading(AngleUnit.DEGREES));

            //Edit for better accuracy

            if ((differenceX1A<0.5) && (differenceY1A<0.5)&& (differenceH1A<1.5)) firstStepRan = true;

            if ((differenceX1B <0.5) && (differenceY1B <0.5)&& (differenceH1B <1.5)) secondStepRan = true;

            if ((differenceX2A <0.5) && (differenceY2A <0.5)&& (differenceH2A <1.5)) thirdStepRan = true;

            if ((differenceX2B <0.5) && (differenceY2B <0.5)&& (differenceH2B <1.5)) fourthStepRan = true;

            if ((differenceX3A <0.5) && (differenceY3A <0.5)&& (differenceH3A <1.5)) fifthStepRan = true;

            if ((differenceX3B <0.5) && (differenceY3B <0.5)&& (differenceH3B <1.5)) sixthStepRan = true;

            if ((differenceX4 <0.5) && (differenceY4 <0.5)&& (differenceH4 <1.5)) seventhStepRan = true;



            if (firstStepRan){
                if (CP1Aresume) {
                    xPower1A = xPower1B;
                    yPower1A = yPower1B;
                    hPower1A = hPower1B;
                }
            }

            if (secondStepRan){
                if (CP1Bresume) {
                    xPower1A = xPower2A;
                    yPower1A = yPower2A;
                    hPower1A = hPower2A;
                }

            }

            if (thirdStepRan){
                if (CP2Aresume) {
                    xPower1A = xPower2B;
                    yPower1A = yPower2B;
                    hPower1A = hPower2B;
                }
            }

            if (fourthStepRan){
                if (CP2Bresume) {
                    xPower1A = xPower3A;
                    yPower1A = yPower3A;
                    hPower1A = hPower3A;
                }
            }

            if (fifthStepRan){
                if (CP3Aresume) {
                    xPower1A = xPower3B;
                    yPower1A = yPower3B;
                    hPower1A = hPower3B;
                }
            }


            if (sixthStepRan){
                if (CP4resume) {
                    xPower1A = xPower4;
                    yPower1A = yPower4;
                    hPower1A = hPower4;
                }
            }

            if (seventhStepRan){
                if (CP5resume) {
                    xPower1A = xPower5;
                    yPower1A = yPower5;
                    hPower1A = hPower5;
                }
            }

            driveFieldCentric(xPower1A, -yPower1A, -hPower1A);








//            sleep(5000);
//            driveFieldCentric(xPower1B, -yPower1B, -hPower1B);


            String data = String.format(Locale.US, "{X: %.1f, Y: %.1f, H: %.1f}", odoPosition.getX(DistanceUnit.INCH), odoPosition.getY(DistanceUnit.INCH), odoPosition.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Delta X:", deltaX1A);
            telemetry.addData("Delta Y:", deltaY1A);
            telemetry.addData("Delta H:", deltaH1A);
            telemetry.addData("firstHasRun", firstStepRan);
            telemetry.addData("differenceX", differenceX1A);
            telemetry.addData("differenceY", differenceY1A);
            telemetry.addData("differenceH", differenceH1A);
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
