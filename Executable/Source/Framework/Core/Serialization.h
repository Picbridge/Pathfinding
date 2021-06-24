/******************************************************************************/
/*!
\file		Serialization.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Serialization declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include  <filesystem>
#include <rapidjson/document.h>
#include "Terrain/Terrain.h"
#include "Misc/PathfindingDetails.hpp"

class PathingTestCase;
class PathingTestData;
class PathingTestResult;
class BehaviorTreePrototype;

class Serialization
{
public:
    static bool initialize();

    static void generate_time_stamp(std::wstringstream &filename);
    static void generate_time_stamp(std::stringstream &filename);

    template <typename T>
    static bool serialize(const T &var, const std::experimental::filesystem::path &filepath);

    template <typename T>
    static bool serialize(const std::vector<T> &vec, const std::experimental::filesystem::path &filepath);

    template <typename T>
    static bool deserialize(T &var, const std::experimental::filesystem::path &filepath);

    static std::experimental::filesystem::path basePath;
    static std::experimental::filesystem::path mapsPath;
    static std::experimental::filesystem::path testsPath;
    static std::experimental::filesystem::path assetsPath;
    static std::experimental::filesystem::path outputPath;
    static std::experimental::filesystem::path behaviorTreePath;

    static bool open_file(FILE **file, const std::experimental::filesystem::path &filepath, bool readOnly);
private:
    
    template <typename T>
    static void serialize_to(rapidjson::Value &val, const T &obj, rapidjson::MemoryPoolAllocator<> &allocator);
    template <typename T>
    static bool deserialize_from(rapidjson::Value &val, T &obj, const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const Terrain::MapData &map, 
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, Terrain::MapData &map,
        const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathingTestCase &test,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, PathingTestCase &test,
        const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathingTestData &data,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, PathingTestData &data,
        const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathingTestResult &result,
        rapidjson::MemoryPoolAllocator<> &allocator);

    static void serialize_to(rapidjson::Value &val, const GridPos &pos,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, GridPos &pos, const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathRequest::Settings &settings,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, PathRequest::Settings &settings,
        const std::wstring &id);

    
    static bool read_file(rapidjson::Document &doc, const std::experimental::filesystem::path &filepath);
    static bool write_file(rapidjson::Document &doc, const std::experimental::filesystem::path &filepath);

    static bool extract_int(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        int &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);

    static bool extract_bool(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        bool &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);

    static bool extract_string(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        std::string &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);

    static bool extract_float(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        float &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);
};

template<typename T>
inline bool Serialization::serialize(const T &var, const std::experimental::filesystem::path &filepath)
{
    rapidjson::Document doc;

    serialize_to(doc, var, doc.GetAllocator());

    return write_file(doc, filepath);
}

template<typename T>
inline bool Serialization::serialize(const std::vector<T> &vec,
    const std::experimental::filesystem::path &filepath)
{
    rapidjson::Document doc;
    doc.SetArray();
    doc.Reserve(static_cast<rapidjson::SizeType>(vec.size()), doc.GetAllocator());

    for (const auto & obj : vec)
    {
        rapidjson::Value temp;
        serialize_to(temp, obj, doc.GetAllocator());

        doc.PushBack(temp, doc.GetAllocator());
    }

    return write_file(doc, filepath);
}

template<typename T>
inline bool Serialization::deserialize(T &var, const std::experimental::filesystem::path &filepath)
{
    rapidjson::Document doc;

    return read_file(doc, filepath) && deserialize_from(doc, var, filepath.wstring());
}

template<typename T>
inline void Serialization::serialize_to(rapidjson::Value &val, const T &obj, rapidjson::MemoryPoolAllocator<> &allocator)
{
    static_assert(std::is_trivial<T>::value, "Non-trivial type can't be read/written as string");

    val.SetString(reinterpret_cast<const char *>(&obj), sizeof(T), allocator);
}

template<typename T>
inline bool Serialization::deserialize_from(rapidjson::Value &val, T &obj, const std::wstring &id)
{
    static_assert(std::is_trivial<T>::value, "Non-trivial type can't be read/written as string");

    if (val.IsString() == false)
    {
        std::wcout << L"Generic serialized object " << id << L" is not a string" << std::endl;
        return false;
    }

    const std::string data = val.GetString();

    memcpy(&obj, data.c_str(), sizeof(T));

    return true;
}
