#pragma once 

#include <string>
#include <vector>

// Only support c++14 or later
namespace dir
{
	int listAllFilesPathsFromDir(const std::string& dirPath,
		const std::string &extension,
		std::vector<std::string> &path);

	bool CheckIfFilePathContainSubPath(const std::string& FilePath,
		const std::string &SubPath);

	void CreateDir(const std::string &dir);


	void MakeDirByName(const std::string &Rootdir, const std::string &folderName, std::string &dirCreated);

	bool CheckFileExist(const std::string &FilePath);

	int GetAllFolderPathInADir(const std::string &Directory, std::vector<std::string> &vfolderpaths);
	int GetAllFolderNameInADir(const std::string &Directory, std::vector<std::string> &vfolderNames);
	int FileCopy(const std::string &FromFilePath, const std::string &ToFilePath);

	int FileRename(const std::string &FromPathName, const std::string &ToPathName);
	void GetBackupFileName(const std::string &FileName, std::string &BackupFileName);
	
}