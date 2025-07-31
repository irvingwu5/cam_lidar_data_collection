#include "FileManager.h"

std::string FileManager::getRootPath()
{
    return root_path;
}

std::vector<std::string> FileManager::getFileChildPaths()
{
    return get_subdirectories(root_path);
}

void FileManager::setSavePath(std::string name)
{

    std::string dir = root_path + name;
    Config::select_path = dir + "/";
}

int FileManager::getPathCount(std::string dir)
{
    std::vector<std::string> current_files = get_subdirectories(dir);
    int number = 0;
    for (const auto& file : current_files)
    {
        number = std::max(number, std::stoi(file));
    }
    return number + 1;
}

std::string FileManager::getSavePath()
{
    std::cout << "SavePath: " << Config::select_path << std::endl;
   return  Config::select_path;
}


std::string FileManager::get_256_lidar_save_path()
{
    std::string full_path = Config::select_path + Config::lidar_256_path + "/Sequences";
    std::cout << "256_lidar_save_path: " << full_path << std::endl;
    return full_path;
}

std::string FileManager::get_64_lidar_save_path()
{
    std::string full_path = Config::select_path + Config::lidar_64_path + "/Sequences";
    std::cout << "64_lidar_save_path: " << full_path << std::endl;
    return full_path;
}

std::string FileManager::get_central_cam_path() {
    std::string full_path = Config::select_path + Config::central_cam_path + "/Sequences";
    std::cout << "\ncentral_cam_save_path: " << full_path << std::endl;
    return full_path;
}

std::string FileManager::get_side_cam_path() {
    std::string full_path = Config::select_path + Config::side_cam_path + "/Sequences";
    std::cout << "\nside_cam_save_path: " << full_path << std::endl;
    return full_path;
}

bool FileManager::createDirectory(const std::string &namefile,bool is_has_root)
{
    std::string dir = root_path + namefile;
    if (!is_has_root){
       dir = namefile;
    }
    std::string cmd = "mkdir -p " + dir;
    return system(cmd.c_str()) == 0;
}

bool FileManager::deleteFile(const std::string &filename)
{
    std::string path = root_path + filename;
    std::string cmd = "rm -r -f " + path;
    std::cout << "deleteFile: " << path << std::endl;
    return system(cmd.c_str()) == 0;
}

bool FileManager::saveImage(const std::string &full_path, const cv::Mat &image) {
    return cv::imwrite(full_path, image);
}

bool FileManager::is_usb_inserted()
{
    const std::string usb_mount_path = get_usb_session_folder();
    struct stat mount_stat, parent_stat;

    // 获取挂载点本身的属性
    if (stat(usb_mount_path.c_str(), &mount_stat) != 0)
        return false;

    // 获取其父目录的属性
    std::string parent_path = usb_mount_path.substr(0, usb_mount_path.find_last_of('/'));
    if (stat(parent_path.c_str(), &parent_stat) != 0)
        return false;

    // 判断是否为挂载点（设备号不同）
    return mount_stat.st_dev != parent_stat.st_dev;
}

std::string FileManager::get_usb_session_folder()
{
    return Config::usb_path;
}

std::string FileManager::get_parent_path(const std::string &path)
{
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? "" : path.substr(0, pos);
}

bool FileManager::is_directory(const std::string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool FileManager::create_directory(const std::string &path)
{
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

bool FileManager::cp_item(const std::string &src, const std::string &dst)
{
    std::string cmd = "cp -r -f \"" + src + "\" \"" + dst + "\" && sync";
    return system(cmd.c_str()) == 0;
}

std::string FileManager::move_folder_contents(std::string &src_folder, const std::string &dst_folder)
{
    if (!is_directory(src_folder))
        return "源目录不存在: " + src_folder;

    if (!is_directory(dst_folder))
        return "目标目录不存在: " + src_folder;

    DIR *dir = opendir(src_folder.c_str());
    if (!dir)
        return "无法打开源目录: " + src_folder + " 请先录制数据";

    src_folder = get_parent_path(get_parent_path(src_folder));
    if (!cp_item(src_folder, dst_folder))
        return "移动失败: " + src_folder + " -> " + dst_folder;

    closedir(dir);
    return "";
}

void FileManager::savePointCloudAsKITTI(const std::vector<RadarPoint>& cloud, std::string oss)
{
    std::ofstream ofs(oss, std::ios::out | std::ios::binary);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to open file for writing: " << oss << std::endl;
        return;
    }
    if (cloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }
    for (const auto& point : cloud) {  
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float intensity = point.intensity;
        float data[4] = {x, y, z, intensity};
        ofs.write(reinterpret_cast<const char *>(data), sizeof(float) * 4);
    }
    ofs.close();
    std::cout << "Saved KITTI point cloud: " << oss << std::endl;
}

std::vector<std::string> FileManager::get_subdirectories(const std::string &path)
{
    std::vector<std::string> folders;
    DIR *dir = opendir(path.c_str());
    if (!dir)
    {
        perror("opendir failed");
        return folders;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string name = entry->d_name;
        if (name == "." || name == "..")
            continue;

        std::string full_path = path + "/" + name;
        struct stat st;
        if (stat(full_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
            folders.push_back(name);
    }

    closedir(dir);
    return folders;
}
