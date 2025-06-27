package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@Config
@Autonomous
public class threeSpecDraft extends LinearOpMode {
    PID_Access pid;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() {

        pid.initialize(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        waitForStart();
        Pose2D CPS = new Pose2D(DistanceUnit.INCH, 16.8,37.8, AngleUnit.DEGREES,0);
        Pose2D CP1A = new Pose2D(DistanceUnit.INCH, 29,22.5, AngleUnit.DEGREES,-33.4);
        Pose2D CP1B = new Pose2D(DistanceUnit.INCH, 9.6, 33.1, AngleUnit.DEGREES, -133.4);
        Pose2D CP2A = new Pose2D(DistanceUnit.INCH, 29.3,11.5,AngleUnit.DEGREES,-33.4);
        Pose2D CP2B = new Pose2D(DistanceUnit.INCH, 13.4,25.1,AngleUnit.DEGREES,-150);
        Pose2D CP3A = new Pose2D(DistanceUnit.INCH, 28.9,1.3,AngleUnit.DEGREES,-33.4);
        Pose2D CP3B = new Pose2D(DistanceUnit.INCH, 13.7,18.8,AngleUnit.DEGREES,-170);
        Pose2D CP4 = new Pose2D(DistanceUnit.INCH, 23.2,22.4,AngleUnit.DEGREES,0);
        Pose2D CP5 = new Pose2D(DistanceUnit.INCH, 49.8,60.5,AngleUnit.DEGREES,0);
        while(opModeIsActive()) {
            pid.odo.update();
            //usually you would do point to point, i.e. cp1 to cp1a,
            //but we tested it, and all movements occur in respect to the starting point (CPS)
            pid.moveToPose(CPS,CP1A);
            sleep(2000);
            pid.moveToPose(CPS,CP1B);
            sleep(2000);
            pid.moveToPose(CPS,CP2A);
            sleep(2000);
            pid.moveToPose(CPS,CP2B);
            sleep(2000);
            pid.moveToPose(CPS,CP3A);
            sleep(2000);
            pid.moveToPose(CPS,CP3B);
            sleep(2000);
            pid.moveToPose(CPS,CP4);
            sleep(2000);
            pid.moveToPose(CPS,CP5);
            sleep(2000);
            pid.moveToPose(CPS,CP4);
            sleep(2000);
            pid.moveToPose(CPS,CP5);
            sleep(2000);
        }
    }

}
