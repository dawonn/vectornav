using System;

namespace VectorNav
{

/// <summary>
/// Provides information about the VectorNav library.
/// </summary>
class Api
{
	/// <summary>
	/// The library's major version.
	/// </summary>
	public const int MajorVersion = 1;

	/// <summary>
	/// The library's minor version.
	/// </summary>
	public const int MinorVersion = 1;

	/// <summary>
	/// The library's patch version.
	/// </summary>
	public const int PatchVersion = 0;

	/// <summary>
	/// The library's revision version.
	/// </summary>
	public const int RevisionVersion = 126;

	/// <summary>
	/// Returns the version of the VectorNav library.
	/// </summary>
	/// <returns>
	/// The VectorNav library version.
	/// </returns>
	static Version GetVersion()
	{
		return new Version(MajorVersion, MinorVersion, PatchVersion, RevisionVersion);
	}
}

}
