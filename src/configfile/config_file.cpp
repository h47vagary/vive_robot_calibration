#include "config_file.h"
#include <fstream>
#include <iostream>

ConfigFile::ConfigFile()
{
    root_ = Json::Value(Json::objectValue);
}

bool ConfigFile::load(const std::string& file_path)
{
    std::ifstream ifs(file_path, std::ifstream::binary);
    if (!ifs.is_open())
        return false;

    Json::CharReaderBuilder reader_builder;
    std::string errs;
    return Json::parseFromStream(reader_builder, ifs, &root_, &errs);
}

bool ConfigFile::save(const std::string& file_path) const
{
    if (!ensure_directory_exists(D_CONFIG_BASE_PATH))
    {
        std::cerr << "Failed to create directory: " << D_CONFIG_BASE_PATH << std::endl;
        return false;
    }

    std::ofstream ofs(file_path);
    if (!ofs.is_open())
    {
        std::cerr << "Failed to is_open" << std::endl;
        return false;
    }

    Json::StreamWriterBuilder writer;
    writer["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> json_writer(writer.newStreamWriter());
    return json_writer->write(root_, &ofs) == 0;
}

void ConfigFile::set(const std::string& key, const std::string& value)
{
    root_[key] = value;
}

std::string ConfigFile::get(const std::string& key) const
{
    return root_.isMember(key) ? root_[key].asString() : "";
}

std::string ConfigFile::get_or(const std::string& key, const std::string& default_val) const
{
    return root_.isMember(key) ? root_[key].asString() : default_val;
}

Json::Value& ConfigFile::root()
{
    return root_;
}

const Json::Value& ConfigFile::root() const
{
    return root_;
}

bool ConfigFile::ensure_directory_exists(const std::string &dir)
{
    if (dir.empty()) return true;  // 当前目录默认存在
#ifdef _WIN32
    return _mkdir(dir.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(dir.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}
