using System.Globalization;

namespace VectorNav.Protocol.Uart
{

class Util
{
	/// <summary>
	/// Parses a float value in a string representation from a VectorNav
	/// sensor.
	/// </summary>
	/// <param name="s">
	/// The string to parse.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static float ParseFloat(string s)
	{
		if (s.Contains("nan") || s.Contains("NAN"))
			return float.NaN;

		return float.Parse(s, UsaCulture);
	}

	/// <summary>
	/// Parses a double value in a string representation from a VectorNav
	/// sensor.
	/// </summary>
	/// <param name="s">
	/// The string to parse.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static double ParseDouble(string s)
	{
		if (s.Contains("nan") || s.Contains("NAN"))
			return double.NaN;

		return double.Parse(s, UsaCulture);
	}

	/// <summary>
	/// Converts a float value to a string value understandable by a VectorNav
	/// sensor.
	/// </summary>
	/// <param name="v">
	/// The value to convert.
	/// </param>
	/// <returns>
	/// The converted string.
	/// </returns>
	public static string ToString(float v)
	{
		return v.ToString(UsaCulture);
	}

	/// <summary>
	/// Converts a double value to a string value understandable by a VectorNav
	/// sensor.
	/// </summary>
	/// <param name="v">
	/// The value to convert.
	/// </param>
	/// <returns>
	/// The converted string.
	/// </returns>
	public static string ToString(double v)
	{
		return v.ToString(UsaCulture);
	}

	private static readonly CultureInfo UsaCulture = new CultureInfo("en-US");
}

}
