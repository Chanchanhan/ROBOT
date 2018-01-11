#pragma once

#include <memory>
#include <vector>


struct OcvYamlConfig_Impl;

class OcvYamlConfig {
public:
    OcvYamlConfig(const std::string &filepath);
    ~OcvYamlConfig();
    std::string text(const std::string &config, const std::string &def = "",
                     bool normalize = false) const;
    double value_d(const std::string &config, const double &def = 0.0) const;
    float value_f(const std::string &config, const float &def = 0.0f) const;
    std::vector<int> vector_int(const std::string &config) const;

private:
    std::unique_ptr<OcvYamlConfig_Impl> m_pimpl;
};


