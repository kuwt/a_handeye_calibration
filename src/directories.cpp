#include <experimental/filesystem>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
namespace fs = std::experimental::filesystem;

namespace dir
{
	/********************************************/
	//              utilities
	
	
	// return the filenames of all files that have the specified extension
	// in the specified directory and all subdirectories
	void get_all_file(const fs::path& root,
		const std::string& ext,
		std::vector<fs::path>& ret)
	{
		if (!fs::exists(root) || !fs::is_directory(root)) return;

		fs::recursive_directory_iterator it(root);
		fs::recursive_directory_iterator endit;

		while (it != endit)
		{
			if (fs::is_regular_file(*it) && it->path().extension() == ext)
			{
				ret.push_back(it->path());
			}
			++it;
		}
	}


	/********************************************/
	//              API

	/**********listAllFilesPathsFromDir***********/
	int listAllFilesPathsFromDir(const std::string& dirPath,
		const std::string &extension,
		std::vector<std::string> &path)
	{
		path.clear();
		std::vector<fs::path> dataPaths;

		{
			std::vector<fs::path> dataPathsPerExt;

			dataPathsPerExt.clear();
			get_all_file(dirPath,
				extension,
				dataPaths);
			dataPaths.insert(dataPaths.end(), dataPathsPerExt.begin(), dataPathsPerExt.end());

		}
		for (unsigned i = 0; i < dataPaths.size(); ++i)
		{
			std::string dataPath = dataPaths.at(i).string();
			path.push_back(dataPath);
		}

		return 0;
	}
	/**********check if a particular string exist in the FIlePath string***********/
	bool CheckIfFilePathContainSubPath(const std::string& FilePath,
		const std::string &SubPath)
	{
		int wildcard_pos = FilePath.find(SubPath);
		if (wildcard_pos != std::string::npos)
		{
			return true;
		}
		else
		{
			return false;
		}

	}

	/**********Create Directory***********/
	void CreateDir(const std::string &dir)
	{
		if (fs::create_directories(dir))
		{
			std::cerr << "Directory Created: " << dir << std::endl;
		}
	}

	/**********Make Directory By FolderName appended with a subindex and the directory is in dirCreated***********/
	void MakeDirByName(const std::string &Rootdir, const std::string &folderName, std::string &dirCreated)
	{
		int proposeDirIdx = 0;
		
		std::string strproposeDirIdx = std::to_string(proposeDirIdx);
		std::string proposeDir = Rootdir + "/" + folderName;
		std::string proposeFullDir = proposeDir + strproposeDirIdx;

		while (fs::exists(proposeFullDir))
		{
			proposeDirIdx++;
			strproposeDirIdx = std::to_string(proposeDirIdx);

			proposeFullDir = proposeDir + strproposeDirIdx;
		}

		std::string NewDir = proposeFullDir;
		CreateDir(NewDir);

		dirCreated = NewDir;
	}
	/**********CheckFileExist***********/
	bool CheckFileExist(const std::string &FilePath)
	{
		if (fs::exists(FilePath))
		{
			return true;
		}
		else
		{
				return false;
		}
	}

	/**********GetAllFolderInADir***********/
	int GetAllFolderPathInADir(const std::string &Directory, std::vector<std::string> &vfolderpaths)
	{
		vfolderpaths.clear();
		if (fs::exists(Directory))
		{
			for (auto& p : fs::recursive_directory_iterator(Directory))
			{
				if (p.status().type() == fs::file_type::directory)
				{
					vfolderpaths.push_back(p.path().string());
				}
			}
			return 0;
		}
		else
		{
			return -1;
		}
	}

	/**********GetAllFolderInADir***********/
	int GetAllFolderNameInADir(const std::string &Directory, std::vector<std::string> &vfolderNames)
	{
		vfolderNames.clear();
		if (fs::exists(Directory))
		{
			for (auto& p : fs::recursive_directory_iterator(Directory))
			{
				if (p.status().type() == fs::file_type::directory)
				{
					
					vfolderNames.push_back(p.path().stem().string());
				}
			}
			return 0;
		}
		else
		{
			return -1;
		}
	}

	/**********FileCopy***********/
	int FileCopy(const std::string &FromFilePath, const std::string &ToFilePath)
	{
		if (!FromFilePath.empty() && !ToFilePath.empty())
		{
			fs::copy(FromFilePath, ToFilePath);
			return 0;
		}
		else
		{
			return -1;
		}
	}

	/**********FileRename***********/
	int FileRename(const std::string &FromPathName, const std::string &ToPathName)
	{
		if (!FromPathName.empty() && !ToPathName.empty())
		{
			fs::rename(FromPathName, ToPathName);
			return 0;
		}
		else
		{
			return -1;
		}
	}
	
	void GetBackupFileName(const std::string &FileName, std::string &BackupFileName)
	{
		int proposeBackupIdx = 1;
		std::string strproposeBackupIdx = std::to_string(proposeBackupIdx);
		std::string tmpBackupFileName = FileName;
		while (fs::exists(tmpBackupFileName))
		{
			strproposeBackupIdx = std::to_string(proposeBackupIdx);

			tmpBackupFileName = FileName + strproposeBackupIdx;
			proposeBackupIdx++;
		}

		BackupFileName = tmpBackupFileName;
	}
}