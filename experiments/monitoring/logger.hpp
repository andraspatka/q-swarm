#ifndef LOGGER
#define LOGGER

#include <string>
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

        static void logPosition(const std::string &footbotId, const std::vector<double>& values) {
            std::string fileName = std::string(Logger::LOG_DIRECTORY) + "/" + footbotId + ".csv";
            std::ofstream file(fileName, std::ios::app);
            if (!file.is_open()) {
                std::cout << "Logger: There was a problem opening the output file!\n";
                exit(1);
            }
            for (int i = 0; i < values.size() - 1; ++i) {
                file << values[i] << ",";
            }
            file << values[values.size() - 1] << "\n";

            file.close();
        }
    };
}
#endif


