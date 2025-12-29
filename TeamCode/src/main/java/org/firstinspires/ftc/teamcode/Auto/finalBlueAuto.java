package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "finalBlueAuto", group = "Autos")
public class finalBlueAuto extends ActionOpMode {

	private static class	MoveTask {
		private PathChain 	_path ;
		private double 		_wait ;
		private int			_state ;

		public boolean		done ;

		MoveTask( PathChain path, double wait )
		{
			_path = path ;
			_wait = wait ;
			_state = 0 ;
			done = false ;
		}

		public void	update()
		{
			switch ( _state )
			{
				case 0:
					follower.followPath( _path, true ) ;
					_state= 1 ;
					break ;
				case 1:
					if ( ! ! ! follower.isBusy() ) {
						pathtimer.resetTimer() ;
						_state = 2 ;
					}
					break ;	
				case 2:
					if ( pathTimer.getElapsedTimeSeconds() > _wait ) {
						_wait= 0. ;
						_state= 3 ;
					}
					break ;
				case 3:
					act() ;
					_state = 4 ;
					break ;
				case 4:
					if ( done() ) {
						_state = 5 ;
					}
				case 5:
					if ( _wait > 0. ) {
						pathtimer.resetTimer() ;
						_state = 6 ;
					}
					else {
						_state = 7 ;
					}
					break ;
				case 6:
					if ( pathTimer.getElapsedTimeSeconds() > _wait ) {
                        _wait= 0. ;
                        _state= 7 ;
                    }
					break ;
				case 7:
					done = true ;
					break ;
			}
		}

		public void	act() { }
		public bool done() { return true ; }
	}

    private static final double PATH_COMPLETION_T = 0.985;

    private Follower follower;
    private Timer pathTimer;

    private Intake intake;
    private Shooter shooter;

    private int currentTaskIndex = 0;
    private int taskPhase = 0; // 0 = DRIVE, 1 = WAIT

	public final double	 shoot112 = Math.toRadians( 112) ;
	public final double	 shoot128 = Math.toRadians( 128) ;
	public final double	 shoot143 = Math.toRadians( 143) ;
	public final double  dir180 = Math.toRadians( 180) ;
	public final double  dir270 = Math.toRadians( 270) ;

    private final Pose start = new Pose(20.903225806451616, 98.9032258064516, Math.toRadians(-36));

    private final Pose shootPreload = new Pose(44.12903225806452, 98.9032258064516, shoot143 );
    private final Pose shootPreloadCtl = new Pose(76.64516129032258, 89.2258064516129);

    private final Pose intake1 = new Pose(14.70967741935484, 87.09677419354838, dir180 );
    private final Pose intake1Ctl = new Pose(49.5483870967742, 72.19354838709677);

    private final Pose shoot1Pose = new Pose(51.67741935483871, 82.06451612903226, shoot128 );

    private final Pose intake2 = new Pose(14.516129032258064, 67.74193548387098, dir180 );
    private final Pose intake2Ctl1 = new Pose(63.096774193548384, 64.45161290322581);
    private final Pose intake2Ctl2 = new Pose(26.516129032258064, 44.32258064516128);

    private final Pose shoot2Pose = new Pose(56.516129032258064, 16.645161290322584, shoot112 );
    private final Pose shoot2Ctl = new Pose(38.32258064516129, 52.25806451612903);

    private final Pose intake3 = new Pose(7.935483870967742, 34.645161290322584, dir180 );
    private final Pose intake3Ctl = new Pose(45.29032258064516, 36.38709677419355);

    private final Pose shoot3Pose = new Pose(56.516129032258064, 16.645161290322584, shoot112 );
    private final Pose shoot3Ctl = new Pose(45.29032258064516, 36.38709677419355);

    private final Pose intake4 = new Pose(8.70967741935484, 7.935483870967735, dir270 );
    private final Pose intake4Ctl = new Pose(7.548387096774194, 33.09677419354839);

    private final Pose shoot4Pose = new Pose(56.70967741935483, 16.451612903225808, shoot112 );
    private final Pose shoot4Ctl = new Pose(27.677419354838708, 26.903225806451605);

	private static class	ShootTask extends MoveTask
	{
		ShootTask( PathChain path ) { super( path, 3. ) ; }
		public void	act() { intake.shoot() ;  _wait= 3. ; }
		public boolean done() {  return intake.enable() ; }
	}

	private static class	IntakeTask extends MoveTask
	{
		IntakeTask( PathChain path ) { super( path, 1.5 ) ; }
		public void act() { intake.enable() ;  _wait= 1. ; }
		public boolean done() { return true ; }
	}

	public class	ChainBuilder
	{
		private  Follower   _fol ;
		private  Pose  _cur ;

		ChainBuilder( Follower fol, Pose pose )
		{
			_fol = fol ;
			_cur = pose ;
		}

		public PathChain	step( Pose ctl, Pose end )
		{
			Pose old = _cur ;
			_cur = end ;

			return follower.pathBuilder()
					.addPath( new BezierCurve( old, ctl, _cur ))
					.setLinearHeadingInterpolation( old.getHeading(), _cur.getHeading())
					.build() ;
		}
		public PathChain	step( Pose ctl1, Post ctl2, Pose end )
		{
			Pose old = _cur ;
			_cur = end ;

			return follower.pathBuilder()
					.addPath( new BezierCurve( old, ctl1, ctl2, _cur ))
					.setLinearHeadingInterpolation( old.getHeading(), _cur.getHeading())
					.build() ;
		}
	}

    public class Steps {

		public	ChainBuilder	build ;
		public  int				step ;

		public void		shoot( PathChain path ) { tasks.add( new ShootTask( path ) ; }
		public void		intake( PathChain path ) { tasks.add( new IntakeTask( path ) ; }

        Steps(Follower follower) {

			build = new ChainBuilder( foll, start ) ;
			step = 0 ;

			shoot( 	build.step( shootPreloadCtl, shootPreload )) ;
			intake( build.step( intake1Ctl, intake1 )) ;
			shoot( 	build.step( intake1Ctl, shoot1Pose )) ;
			intake( build.step( intake2Ctl1, intake2Ctl2, intake2 )) ;
            shoot( 	build.step( shoot2Ctl, shoot2Pose)) ;
            intake( build.step( intake3Ctl, intake3)) ;
            shoot( 	build.step( shoot3Ctl, shoot3Pose)) ;
            intake( build.step( intake4Ctl, intake4 )) ;
            shoot( 	build.step( shoot4Ctl, shoot4Pose )) ;
        }

		private final List<MoveTask>	tasks ;

		public MoveTask		update()
		{
			if ( step > tasks.size() ) {
				return ;
			}

			MoveTask task = tasks.get( step ) ;

			task.update() ;
			if ( task.done ) { step += 1 ; }
		}
    }

	public Steps steps ;

    @Override
    public void init() {

        pathTimer = new Timer();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, shooter ) ;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        paths = new Paths(follower);
        steps = new Steps( follower ) ;
    }

    @Override
    public void start() {
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        steps.update() ;
    }
}
