package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveCommand extends Command{
    private double x, y;
    private SwerveDriveSubsystem drive;
    private Command command;

    // This command drives some distance forward and sideways
    public AutoDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, double x, double y) {
        this.x = x;
        this.y = y;

        drive = swerveDriveSubsystem;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        // Get our current pose
        Pose2d currentPose = drive.getPose();

        Pose2d startPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(x, y)), new Rotation2d());

        // Create a list of points 
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);
        
        // Create the autonomous path
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(
                4.0, 
                4.0,
                Units.degreesToRadians(360), 
                Units.degreesToRadians(540)
                ), 
            // Our desired end state is not moving and facing the same
            new GoalEndState(0.0, new Rotation2d())
        );

        // Don't flip the path
        path.preventFlipping = true;
        
        // TODO: Path doesn't finish
        // Schedule the command to follow the path
        command = new ParallelDeadlineGroup(new WaitCommand(2), AutoBuilder.followPath(path));
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override 
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
