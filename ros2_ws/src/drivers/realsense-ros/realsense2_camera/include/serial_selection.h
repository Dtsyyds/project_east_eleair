#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace realsense2_camera
{
// An immutable selection survives replacement of the active device handle.
class SerialSelection
{
public:
    explicit SerialSelection(std::string value = "")
    {
        if (value.empty()) return;
        std::size_t start = 0;
        do
        {
            const auto end = value.find(',', start);
            auto token = value.substr(start, end == std::string::npos ? end : end - start);
            const auto first = token.find_first_not_of(" \t\r\n");
            const auto last = token.find_last_not_of(" \t\r\n");
            token = first == std::string::npos ? "" : token.substr(first, last - first + 1);
            if (!token.empty() && token.front() == '_') token.erase(0, 1);
            if (token.empty()) throw std::invalid_argument("serial_no contains an empty serial number");
            if (std::find(_serials.begin(), _serials.end(), token) == _serials.end())
                _serials.push_back(token);
            if (end == std::string::npos) break;
            start = end + 1;
        } while (true);
    }

    bool accepts(const std::string &serial) const
    {
        return _serials.empty() || std::find(_serials.begin(), _serials.end(), serial) != _serials.end();
    }

    const std::vector<std::string> &serials() const { return _serials; }

private:
    std::vector<std::string> _serials;
};
}
