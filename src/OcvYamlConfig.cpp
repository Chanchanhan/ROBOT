#include <cctype>
#include <algorithm>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>


#include "OcvYamlConfig.h"

struct OcvYamlConfig_Impl {
    cv::Ptr<cv::FileStorage> fs;
};


OcvYamlConfig::OcvYamlConfig(const std::string &filepath) {
    m_pimpl = std::make_unique<OcvYamlConfig_Impl>();
    m_pimpl->fs = new cv::FileStorage(filepath, cv::FileStorage::READ);
    if (!m_pimpl->fs->isOpened()) {
        std::cerr << "Cannot open config file: " << filepath << std::endl;
    }
}

OcvYamlConfig::~OcvYamlConfig() = default;

std::string OcvYamlConfig::text(const std::string &config,
                                const std::string &def, bool normalize) const {
    const cv::FileNode &n = (*(m_pimpl->fs))[config];
    if (n.isNone()) {
        std::cerr << "Caution : can not find text " << config << std::endl;
        return def;
    } else {
        std::string result = (std::string)n;
        if (normalize) {
            std::transform(result.begin(), result.end(), result.begin(), ::tolower);
        }
        return result;
    }
}

double OcvYamlConfig::value_d(const std::string &config,
                              const double &def) const {
    const cv::FileNode &n = (*(m_pimpl->fs))[config];
    if (n.isNone()) {
        std::cerr << "Caution : can not find value_d " << config << std::endl;
        return def;
    } else {
        return (double)n;
    }
}

float OcvYamlConfig::value_f(const std::string &config,
                             const float &def) const {
    const cv::FileNode &n = (*(m_pimpl->fs))[config];
    if (n.isNone()) {
        std::cerr << "Caution : can not find value_f " << config << std::endl;
        return def;
    } else {
        return double(n);
    }
}

std::vector<int> OcvYamlConfig::vector_int(const std::string &config) const {
    const cv::FileNode &n = (*(m_pimpl->fs))[config];
    if (n.isNone()) {
        std::cerr << "Caution : can not find vector_int " << config << std::endl;
        return std::vector<int>();
    } else {
        std::vector<int> values;
        for (auto it = n.begin(); it < n.end(); ++it) {
            values.emplace_back(*it);
        }
        return values;
    }
}
