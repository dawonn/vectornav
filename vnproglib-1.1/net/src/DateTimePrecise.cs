using System;
using System.Diagnostics;

namespace VectorNav
{

/// <summary>
/// Provides precise timing operations.
/// </summary>
public class DateTimePrecise
{
	private sealed class DateTimePreciseSafeImmutable
	{
		internal DateTimePreciseSafeImmutable(DateTime t_observed, DateTime t_base,
				long s_observed, long stopWatchFrequency)
		{
			_t_observed = t_observed;
			_t_base = t_base;
			_s_observed = s_observed;
			_stopWatchFrequency = stopWatchFrequency;
		}
		internal readonly DateTime _t_observed;
		internal readonly DateTime _t_base;
		internal readonly long _s_observed;
		internal readonly long _stopWatchFrequency;
	}

	/// <summary>
	/// Gets a <seealso cref="DateTime"/> object that is set to the current
	/// precise date and time on this computer, expressed as the Coordinate
	/// Universal Time (UTC).
	/// </summary>
	public static DateTime UtcNow { get { return StaticDtp.UtcNowCustom; } }

	/// <summary>
	/// Gets a <seealso cref="DateTime"/> object that is set to the current
	/// precise date and time on this computer, expressed as the local time.
	/// </summary>
	public static DateTime Now { get { return StaticDtp.NowCustom; } }
	
	/// <summary>
	/// Creates a new instance of <see cref="DateTimePrecise"/>.
	/// </summary>
	private DateTimePrecise() : this(10) { }

	/// <summary>
	/// Creates a new instance of <see cref="DateTimePrecise"/>.
	/// </summary>
	/// <param name="synchronizePeriodSeconds">
	/// The number of seconds after which the <see cref="DateTimePrecise"/>
	/// will synchronize itself with the system clock. A large value may
	/// cause arithmetic overflow exceptions to be thrown. A small value may
	/// cause the time to be unstable. A good default value is 10.
	/// </param>
	private DateTimePrecise(long synchronizePeriodSeconds)
	{
		_stopwatch = Stopwatch.StartNew();
		_stopwatch.Start();

		var t = DateTime.UtcNow;
		_immutable = new DateTimePreciseSafeImmutable(t, t, _stopwatch.ElapsedTicks,
			Stopwatch.Frequency);

		//_synchronizePeriodSeconds = synchronizePeriodSeconds;
		_synchronizePeriodStopwatchTicks = synchronizePeriodSeconds *
			Stopwatch.Frequency;
		//_synchronizePeriodClockTicks = synchronizePeriodSeconds *
		//	ClockTickFrequency;
	}

	/// Returns the current date and time, just like DateTime.UtcNow.
	private DateTime UtcNowCustom
	{
		get
		{
			var s = _stopwatch.ElapsedTicks;
			var immutable = _immutable;

			if (s < immutable._s_observed + _synchronizePeriodStopwatchTicks)
			{
				return immutable._t_base.AddTicks(((
					s - immutable._s_observed) * ClockTickFrequency) / (
					immutable._stopWatchFrequency));
			}
			else
			{
				var t = DateTime.UtcNow;

				var tBaseNew = immutable._t_base.AddTicks(((
					s - immutable._s_observed) * ClockTickFrequency) / (
					immutable._stopWatchFrequency));

				_immutable = new DateTimePreciseSafeImmutable(
					t,
					tBaseNew,
					s,
					((s - immutable._s_observed) * ClockTickFrequency * 2)
					/
					(t.Ticks - immutable._t_observed.Ticks + t.Ticks +
						t.Ticks - tBaseNew.Ticks - immutable._t_observed.Ticks)
				);

				return tBaseNew;
			}
		}
	}

	/// <summary>
	/// Returns the current date and time, just like DateTime.Now.
	/// </summary>
	private DateTime NowCustom
	{
		get { return UtcNowCustom.ToLocalTime(); }
	}

	private static long _synchronizePeriodStopwatchTicks;
	//private long _synchronizePeriodSeconds;
	//private long _synchronizePeriodClockTicks;
	private const long ClockTickFrequency = 10000000;
	private static DateTimePreciseSafeImmutable _immutable;
	private readonly Stopwatch _stopwatch;

	private static readonly DateTimePrecise StaticDtp = new DateTimePrecise();
}

}
