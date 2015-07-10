#pragma once
////todo finish file searching

#include <io.h>
#include <string>
#include <vector>

inline int exRand()
{
	return rand()*RAND_MAX+rand();
}
//注意：当字符串为空时，也会返回一个空字符串
void split(const std::string s, const std::string delim,std::vector<std::string> &ret)
{
	size_t last = 0;
	size_t index=s.find_first_of(delim,last);
	while (index!=std::string::npos)
	{
		ret.push_back(s.substr(last,index-last));
		last=index+1;
		index=s.find_first_of(delim,last);
	}
	if (index-last>0)
	{
		ret.push_back(s.substr(last,index-last));
	}
}
//注意：当字符串为空时，也会返回一个空字符串
std::vector<std::string>  split(const std::string s, const std::string delim)
{
	std::vector<std::string> ret;
	split(s,delim,ret);
	return ret;
}
/*
dir should ends with '\\'
*/
int get_filenames(const std::string& dir, std::vector<std::string>& filenames)
{
	struct _finddata_t fa;
	long fhandle;
	if((fhandle = _findfirst(dir.c_str(),&fa))==-1L)
	{
		return 0;
	}
	else
	{
		do{			
			//filenames.push_back(split(string(fa.name),".")[0]);
		}while(_findnext(fhandle,&fa)==0);
		_findclose(fhandle);
		return filenames.size();
	}
}
//#include <windows.h>
//#include "boost\filesystem.hpp"
//namespace fs = boost::filesystem;
//fs::path dir_path = "directory/of/yourown/";
//fs::diretory_iterator lDirIter;
//for(direcotry_iterator itr(dir_path);
//    itr != end_itr;
//    ++itr)
//{
//	
//// is_directory 判断是否是目录
//// is_regular_file 判断是否是文件
//}
//int get_filenames(const std::string& dir, std::vector<std::string>& filenames)  
//{  
//    fs::path path(dir);  
//    if (!fs::exists(path))  
//    {  
//        return -1;  
//    }  
//  
//    fs::directory_iterator end_iter;  
//    for (fs::directory_iterator iter(path); iter!=end_iter; ++iter)  
//    {  
//        if (fs::is_regular_file(iter->status()))  
//        {  
//            filenames.push_back(iter->path().string());  
//        }  
//  //don't need directory
//       /* if (fs::is_directory(iter->status()))  
//        {  
//            get_filenames(iter->path().string(), filenames);  
//        } */ 
//    }  
//  
//    return filenames.size();  
//}  
//
//int find_all_files(const char * lpPath)
//{
//    char szFind[MAX_PATH];
//    WIN32_FIND_DATA FindFileData;
//    strcpy(szFind,lpPath);
//    strcat(szFind,"\\*.*");
//    HANDLE hFind=::FindFirstFile(szFind,&FindFileData);
//    if(INVALID_HANDLE_VALUE == hFind)   
//        return -1;
//    
//    do
//    {
//        if(FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
//        {
//            if(strcmp(FindFileData.cFileName,".")!=0 && strcmp(FindFileData.cFileName, "..")!=0)
//            {
//                //发现子目录，递归之
//                char szFile[MAX_PATH] = {0};
//                strcpy(szFile,lpPath);
//                strcat(szFile,"\\");
//                strcat(szFile,FindFileData.cFileName);
//                find_all_files(szFile);
//            }
//        }
//        else
//        {
//            //找到文件，处理之
//            std::cout << lpPath << "\\" << FindFileData.cFileName << std::endl;
//        }
//    }while(::FindNextFile(hFind,&FindFileData));
//
//    ::FindClose(hFind);
//    
//    return 0;
//}