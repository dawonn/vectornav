#if defined WIN32 && defined DLL_VALIDATOR

#include "dllvalidator.h"

#include <Windows.h>

void dumpImportDirectory32(PeLib::PeFile32& pef, std::vector<std::string>& dllNamesOut)
{
	if (pef.readImportDirectory() == PeLib::ERROR_NO_ERROR)
	{
		const PeLib::ImportDirectory32& imp = static_cast<PeLib::PeFile32&>(pef).impDir();

		for (unsigned int i = 0; i < imp.getNumberOfFiles(PeLib::OLDDIR); i++)
		{
			dllNamesOut.push_back(imp.getFileName(i, PeLib::OLDDIR));
		}
	}
}

void dumpImportDirectory64(PeLib::PeFile64& pef, std::vector<std::string>& dllNamesOut)
{
	if (pef.readImportDirectory() == PeLib::ERROR_NO_ERROR)
	{
		const PeLib::ImportDirectory64& imp = static_cast<PeLib::PeFile64&>(pef).impDir();

		for (unsigned int i = 0; i < imp.getNumberOfFiles(PeLib::OLDDIR); i++)
		{
			dllNamesOut.push_back(imp.getFileName(i, PeLib::OLDDIR));
		}
	}
}

void DllValidator::DllValidatorVisitor::callback(PeLib::PeFile32 &file)
{
	dumpImportDirectory32(file, mRequiredDlls);
}

void DllValidator::DllValidatorVisitor::callback(PeLib::PeFile64 &file)
{
	dumpImportDirectory64(file, mRequiredDlls);
}

DllValidator::DllValidator()
{
}

DllValidator::DllValidator(std::string dllName, std::string currentDirectory) :
	mIsInitialized(false),
	mIsValid(false),
	mPeFile(NULL),
	mFileName(dllName),
	mWorkingDirectory(currentDirectory)
{
}

bool DllValidator::initialize()
{
	mPeFile = PeLib::openPeFile(mFileName);

	if (mPeFile != NULL)
	{
		if ((mPeFile->readMzHeader() == 0) &&
			(mPeFile->readPeHeader() == 0))
		{
			mIsInitialized = true;
		}
	}

	return mIsInitialized;
}

bool DllValidator::hasDllNames()
{
	return mVisitor.mRequiredDlls.size() > 0;
}

void DllValidator::getDllNames(std::vector<std::string>& dllNamesOut)
{
	dllNamesOut.clear();
	dllNamesOut = mVisitor.mRequiredDlls;
}

void DllValidator::getMissingDllNames(std::vector<std::string>& missingDllNamesOut)
{
	missingDllNamesOut.clear();
	missingDllNamesOut = mMissingDlls;
}

bool DllValidator::validate()
{
	mIsValid = false;

	if (mIsInitialized)
	{
		SetCurrentDirectory(mWorkingDirectory.c_str());

		mMissingDlls.clear();

		mPeFile->visit(mVisitor);

		for (size_t index = 0; index < mVisitor.mRequiredDlls.size(); index++)
		{
			HMODULE module = LoadLibraryEx(mVisitor.mRequiredDlls[index].c_str(), NULL, LOAD_LIBRARY_AS_IMAGE_RESOURCE);
			if (NULL == module)
			{
				mMissingDlls.push_back(mVisitor.mRequiredDlls[index]);
			}
		}
	}
	else if (mPeFile == NULL)
	{
		mMissingDlls.push_back(mFileName + " is not a valid DLL file.");
	}

	if (mMissingDlls.empty())
	{
		mIsValid = true;
	}

	return mIsValid;
}

#endif
