#ifndef LOGGER
#define LOGGER

#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#define assertm(exp, msg) assert(((void)msg, exp))

namespace ql {

    class Logger {
    private:
        Logger() = default;
        static constexpr const char* LOG_DIRECTORY = "logs";
    public:

        static void clearMyLogs(const std::string &footbotId) {
            std::string fileName = std::string(Logger::LOG_DIRECTORY) + "/" + footbotId + ".csv";
            std::remove(fileName.c_str());
        }

        static void logPositionStateAndAction(double positionX, double positionY, const std::string& stateName, const std::string& actionName, const std::string &id) {
            std::vector<std::string> toLog = {
                    std::to_string(positionX),
                    std::to_string(positionY),
                    stateName,
                    actionName
            };
            log(id, toLog);
        }

        static void log(const std::string &footbotId, std::vector<std::string> values, bool isDiseaseSim = false) {
            std::string fileName;
            if (isDiseaseSim) {
                fileName = std::string(Logger::LOG_DIRECTORY) + "/disease/" + footbotId + ".csv";
            } else {
                fileName = std::string(Logger::LOG_DIRECTORY) + "/" + footbotId + ".csv";
            }

            std::ofstream file(fileName, std::ios::app);
            if (!file.is_open()) {
                std::cout << "Logger: There was a problem opening the output file!\n";
                exit(1);
            }
            for (int i = 0; i < values.size() - 1; ++i) {
                std::replace(values[i].begin(), values[i].end(), ',', '.');
                file << values[i] << ",";
            }
            file << values[values.size() - 1] << "\n";

            file.close();
        }
    };
}
#endif


