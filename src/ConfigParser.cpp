/// FicTrac http://rjdmoore.net/fictrac/
/// \file       ConfigParser.cpp
/// \brief      Read/write simple key/value pair config files.
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#include "ConfigParser.h"

#include "Logger.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>

using std::string;
using std::vector;

///
/// Default constructor.
///
ConfigParser::ConfigParser()
{
}

///
/// Construct and parse given config file.
///
ConfigParser::ConfigParser(string fn)
{
    read(fn);
}

///
/// Default destructor.
///
ConfigParser::~ConfigParser()
{
}

///
/// Read in and parse specified config file.
///
int ConfigParser::read(string fn)
{
    /// Open input file
    std::ifstream f(fn);
    if (!f.is_open()) {
        LOG_ERR("Could not open config file (%s) for reading!", fn);
        return -1;
    }
    
    _fn = fn;   // save file name for debugging

    /// Parse to map
    string line;
    _data.clear();
    while (getline(f,line)) {
        if ((line[0] == '#') || (line[0] == '%')) { continue; }   // skip comment lines

        /// Tokenise
        const string whitespace = ", \t\n";
        std::size_t delim = line.find(":");
        if (delim >= line.size()) { continue; } // skip blank lines
        string key = line.substr(0, line.find_last_not_of(whitespace,delim-1)+1);
        string val = line.substr(line.find_first_not_of(whitespace,delim+1));

        /// Add to map
        _data[key] = val;

        LOG_DBG("Extracted key: %s  val: %s", key, val);
    }

    /// Clean up
    f.close();
    return static_cast<int>(_data.size());
}

///
/// Write specified map to file.
///
int ConfigParser::write(string fn, std::map<string,string>& map_to_write)
{
    /// Open output file
    std::ofstream f(fn);
    if (!f.is_open()) {
        LOG_ERR("Could not open config file %s for writing!", fn);
        return -1;
    }
    
    /// Write header string
    f << "# FicTrac config file (build " << __DATE__ << ")" << std::endl;

    /// Write map
    static char tmps[4096];
    for (auto& it : map_to_write) {
        // warning: super long str vals will cause overwrite error!
        try { sprintf(tmps, "%-12s : %s\n", it.first.c_str(), it.second.c_str()); }
        catch (std::exception& e) {
			LOG_ERR("Error writing key/value pair (%s : %s)! Error was: %s", it.first, it.second, e.what());
            f.close();
            return -1;
        }
        f << tmps;
    }

    /// Clean up
    int nbytes = static_cast<int>(f.tellp());
    f.close();

	LOG_DBG("Wrote %d bytes to disk!", nbytes);

    return nbytes;
}

///
/// Retrieve string value corresponding to specified key from map.
///
bool ConfigParser::getStr(string key, string& val) {
    auto it = _data.find(key);
    if (it != _data.end()) {
        val = _data[key];
        return true;
    }
    LOG_WRN("Could not find specified key (%s)!", key);
    return false;
}

///
/// Retrieve int value corresponding to specified key from map.
///
bool ConfigParser::getInt(string key, int& val) {
    string str;
    if (getStr(key, str)) {
        try { val = stoi(str); }
        catch (std::exception& e) {
            LOG_ERR("Error parsing config file value (%s : %s) as INT! Error was: %s", key, str, e.what());
            return false;
        }
        return true;
    }
    return false;
}

///
/// Retrieve double value corresponding to specified key from map.
///
bool ConfigParser::getDbl(string key, double& val) {
    string str;
    if (getStr(key, str)) {
        try { val = stod(str); }
        catch (std::exception& e) {
			LOG_ERR("Error parsing config file value (%s : %s) as DBL! Error was: %s", key, str, e.what());
            return false;
        }
        return true;
    }
    return false;
}

///
/// Retrieve vector of int values corresponding to specified key from map.
///
bool ConfigParser::getVecInt(std::string key, vector<int>& val) {
    /// Get value string.
    string str;
    const string whitespace = ", \t\n";
    if (getStr(key, str)) {
        val.clear();
        
        // start array from opening bracket
        size_t begin, end = str.find_first_of("{");
        while (end != string::npos) {
            // extract value
            begin = str.find_first_not_of(whitespace, end+1);
            end = str.find_first_of(whitespace, begin);
            string s = str.substr(begin,end-begin);
            
            // break when we hit closing bracket
            if (s.substr(0,1) == "}") { break; }
            try { val.push_back(stoi(s)); }
            catch (std::exception& e) {
				LOG_ERR("Error parsing config file value (%s : %s) as INT! Error was: %s", key, s, e.what());
                return false;
            }
        }
        return true;
    }
    return false;
}

///
/// Retrieve vector of double values corresponding to specified key from map.
///
bool ConfigParser::getVecDbl(std::string key, vector<double>& val) {
    /// Get value string.
    string str;
    const string whitespace = ", \t\n";
    if (getStr(key, str)) {
        val.clear();
        
        // start array from opening bracket
        size_t begin, end = str.find_first_of("{");
        while (end != string::npos) {
            // extract value
            begin = str.find_first_not_of(whitespace, end+1);
            end = str.find_first_of(whitespace, begin);
            string s = str.substr(begin,end-begin);
            
            // break when we hit closing bracket
            if (s.substr(0,1) == "}") { break; }
            try { val.push_back(stod(s)); }
            catch (std::exception& e) {
				LOG_ERR("Error parsing config file value (%s : %s) as DBL! Error was: %s", e.what(), key, s);
                return false;
            }
        }
        return true;
    }
    return false;
}

///
/// Retrieve vector of vector of int values corresponding to specified key from map.
///
bool ConfigParser::getVVecInt(std::string key, vector<vector<int> >& val) {
    /// Get value string.
    string str;
    const string whitespace = ", \t\n";
    if (getStr(key, str)) {
        val.clear();
        
        // start array from opening bracket
        size_t begin, end = str.find_first_of("{");
        while (end != string::npos) {
            // extract poly
            vector<int> poly;
            
            // start array from opening bracket
            end = str.find_first_of("{", end+1);
            while (end != string::npos) {
                // extract value
                begin = str.find_first_not_of(whitespace, end+1);
                end = str.find_first_of(whitespace, begin);
                string s = str.substr(begin,end-begin);
                
                // break when we hit closing bracket
                if (s.substr(0,1) == "}") { break; }
                try { poly.push_back(stoi(s)); }
                catch (std::exception& e) {
					LOG_ERR("Error parsing config file value (%s : %s) as INT! Error was: %s", key, s, e.what());
                    return false;
                }
            }
            val.push_back(poly);
        }
        return true;
    }
    return false;
}

///
/// Print all key/value pairs to stdout.
///
void ConfigParser::printAll()
{
    printf("Config file (%s):\n", _fn.c_str());
    
    std::stringstream s;
    for (auto& it : _data) {
        s << "\t" << it.first << "\t: " << it.second << std::endl;
    }
    std::cout << s.str();
}
