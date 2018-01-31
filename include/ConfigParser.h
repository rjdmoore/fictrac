/// FicTrac http://rjdmoore.net/fictrac/
/// \file       ConfigParser.h
/// \brief      Read/write simple key/value pair config files.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include <map>
#include <string>
#include <vector>

class ConfigParser
{
public:
    ConfigParser();
    ConfigParser(std::string fn);
    ~ConfigParser();

    /// File IO
    int read(std::string fn);
    int write(std::string fn, std::map<std::string, std::string>& pmap);    // write supplied map to file
    int write(std::string fn) { return write(fn, _string_data); }
    int write() { return write(_fn); }

    /// Accessor functions
    bool getStr(std::string key, std::string& val);
    bool getInt(std::string key, int& val);
    bool getDbl(std::string key, double& val);
    bool getVecInt(std::string key, std::vector<int>& val);
    bool getVecDbl(std::string key, std::vector<double>& val);
    bool getVVecInt(std::string key, std::vector<std::vector<int> >& val);

    /// Write access
    template<typename T>
    void add(std::string key, T& val) { _string_data[key] = std::to_string(val); }

    // special case: string
    void add(std::string key, std::string val) { _string_data[key] = val; }
    
    // special case: vector
    template<typename T>
    void add(std::string key, std::vector<T>& val) {
        std::string str = "{ ";
        for (auto v : val) {
            str += std::to_string(v) + ", ";
        }
        str = str.substr(0, std::max(static_cast<int>(str.size()-2),2)) + " }";  // drop last comma
        _string_data[key] = str;
    }
    
    // super special case: vector of vectors
    template<typename T>
    void add(std::string key, std::vector<std::vector<T> >& val) {
        std::string str = "{ ";
        for (auto v : val) {
            str += "{ ";
            for (auto vv : v) {
                str += std::to_string(vv) + ", ";
            }
            str = str.substr(0, str.size()-2) + " }, ";   // drop last comma
        }
        str = str.substr(0, std::max(static_cast<int>(str.size()-2),2)) + " }";   // drop last comma
        _string_data[key] = str;
    }
    
    /// Debugging
    void printAll();

private:
    std::string _fn;                                // keep track of config filename
    std::map<std::string,std::string> _string_data; // string data written/read to/from file
};
