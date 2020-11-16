#ifndef _DLLVALIDATOR_H_
#define _DLLVALIDATOR_H_

#if defined WIN32 && defined DLL_VALIDATOR

#include <string>
#include <vector>

#if defined(_MSC_VER)
	// Disable a bunch of warnings from 3rd party library.
	#pragma warning(push)
	#pragma warning(disable:4091)
	#pragma warning(disable:4251)
	#pragma warning(disable:4275)
#endif

#include "PeLib.h"

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

// Class to validate the input DLL exists and that all of it's first level
// dependencies exist as well.
class DllValidator
{
public:
	DllValidator(std::string dllName, std::string currentDirectory);

	bool initialize();

	bool hasDllNames();

	void getDllNames(std::vector<std::string>& dllNamesOut);

	void getMissingDllNames(std::vector<std::string>& missingDllNamesOut);

	bool validate();

private:
	struct DllValidatorVisitor : public PeLib::PeFileVisitor
	{
		//template<int bits>
		//void dumpImportDirectory(PeLib::PeFile& pef, std::vector<std::string>& dllNamesOut);

		virtual void callback(PeLib::PeFile32 &file);

		virtual void callback(PeLib::PeFile64 &file);

		std::vector<std::string> mRequiredDlls;
	};

	DllValidator();

	bool mIsInitialized;
	bool mIsValid;
	DllValidatorVisitor mVisitor;
	PeLib::PeFile* mPeFile;
	std::string mFileName;
	std::string mWorkingDirectory;
	std::vector<std::string> mMissingDlls;
};

#endif

#endif
