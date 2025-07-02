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
    std::ofstream ofs(file_path);
    if (!ofs.is_open())
        return false;

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
